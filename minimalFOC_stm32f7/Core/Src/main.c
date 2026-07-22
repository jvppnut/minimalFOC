/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "motor/foc_motor.h"
#include "foc_config.h"
#include "foc.h"
#include "driver/foc_drv8323.h"
#include "driver/foc_as5147.h"
#include "core/math/foc_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5147_CSN_PORT      GPIOC
#define AS5147_CSN_PIN       GPIO_PIN_4
#define DRV8323_NSCS_PORT    GPIOA
#define DRV8323_NSCS_PIN     GPIO_PIN_15
#define DRV8323_EN_PORT      GPIOB
#define DRV8323_EN_PIN       GPIO_PIN_13
#define DRV8323_NFAULT_PORT  GPIOC
#define DRV8323_NFAULT_PIN   GPIO_PIN_12
#define DRV8323_INLX_PORT    GPIOC
#define DRV8323_INLX_PIN     GPIO_PIN_9
#define LED_DEBUG_PORT       GPIOB
#define LED_DEBUG_PIN        GPIO_PIN_12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
FOC_Motor_t motor;

static uint16_t          as5147_tx = AS5147_CMD_READ_ANGLECOM;
static volatile uint16_t as5147_rx = 0;
static volatile uint32_t isr_cycles = 0;

/* Binary data logger — 64-byte frames at 1 kHz via UART DMA.
 * Frame: [0xAA][0x55][15 × float32 LE, 60 B][uint8 mode][XOR CRC]
 * Fields: theta_mech, omega_mech, theta_elec, i_u, i_v, i_w,
 *         i_d, i_q, v_d, v_q, i_d_ref, i_q_ref, omega_ref, theta_ref, isr_us */
#define LOG_DECIMATE    20u
#define LOG_FRAME_BYTES 64u

static uint8_t          log_buf[2][LOG_FRAME_BYTES];
static volatile uint8_t log_wr   = 0u;
static volatile uint8_t log_pend = 0u;

/* ISR operating mode */
typedef enum { ISR_MODE_ADC_CAL, ISR_MODE_CONTROL } ISR_Mode_t;
static volatile ISR_Mode_t isr_mode = ISR_MODE_ADC_CAL;

/* ADC zero-current offset calibration */
#define ADC_CAL_SAMPLES 256u
static uint32_t          adc_zero[3];
static uint32_t          adc_cal_sum[3];
static uint32_t          adc_cal_count;
static volatile uint8_t  adc_cal_finished = 0u;

/* Function generator — runs in ISR, drives the active-mode reference */
typedef struct {
    uint8_t  enabled;
    uint8_t  waveform;   /* 0=step  1=square  2=triangle  3=sine  4=staircase2 */
    float    frequency;  /* Hz */
    float    amplitude;  /* peak magnitude */
    float    offset;     /* DC bias */
    float    phase;      /* accumulator [0, 1) */
} FOC_FuncGen_t;
static FOC_FuncGen_t fgen;

/* UART RX — DMA circular buffer; main loop polls write pointer, no callbacks needed */
#define CMD_DMA_BUF_SIZE 64u
static          uint8_t  cmd_dma_buf[CMD_DMA_BUF_SIZE];
static          uint8_t  cmd_dma_rd  = 0u;

/* Mailbox: main loop parses frames and posts here; ISR applies at top of tick */
static volatile uint8_t  pending_cmd  = 0u;
static volatile float    pending_val  = 0.0f;
static volatile uint8_t  cmd_ready    = 0u;
static volatile uint8_t          motor_stopped    = 0u;
static          FOC_CtrlMode_t   mode_before_stop = FOC_MODE_VOLTAGE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
static void PWM_SetDuties(float duty_u, float duty_v, float duty_w);
static void cmd_parse(void);
static void cmd_apply(uint8_t cmd, float val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
        ITM_SendChar(ptr[i]);
    return len;
}

static void DRV8323_SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    uint16_t tx16 = ((uint16_t)tx[0] << 8) | tx[1];
    uint16_t rx16 = 0;
    HAL_GPIO_WritePin(DRV8323_NSCS_PORT, DRV8323_NSCS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx16, (uint8_t*)&rx16, 1, 10);
    HAL_GPIO_WritePin(DRV8323_NSCS_PORT, DRV8323_NSCS_PIN, GPIO_PIN_SET);
    if (rx) { rx[0] = (uint8_t)(rx16 >> 8); rx[1] = (uint8_t)(rx16 & 0xFF); }
}

static uint16_t DRV8323_ReadReg(uint8_t addr)
{
    uint16_t frame = (1u << 15u) | ((uint16_t)(addr & 0x0Fu) << 11u);
    uint8_t tx[2] = { (uint8_t)(frame >> 8), (uint8_t)(frame & 0xFF) };
    uint8_t rx[2] = { 0, 0 };
    DRV8323_SPI_Transfer(tx, rx, 2);
    return ((uint16_t)rx[0] << 8 | rx[1]) & 0x7FFu;
}

static void PWM_SetDuties(float duty_u, float duty_v, float duty_w)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_u * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_v * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_w * period));
}

static void log_pack(void)
{
    if (log_pend) return;

    uint8_t *p = log_buf[log_wr];
    p[0] = 0xAAu;
    p[1] = 0x55u;

    const float payload[15] = {
        motor.state.theta_mech,
        motor.state.omega_mech,
        motor.state.theta_elec,
        motor.state.i_u,
        motor.state.i_v,
        motor.state.i_w,
        motor.state.i_d,
        motor.state.i_q,
        motor.out.v_d,
        motor.out.v_q,
        motor.ref.i_d_ref,
        motor.ref.i_q_ref,
        motor.ref.omega_ref,
        motor.ref.theta_ref,
        (float)isr_cycles / 180.0f
    };
    memcpy(p + 2u, payload, 60u);

    p[62u] = (uint8_t)motor.ref.mode;

    uint8_t crc = 0u;
    for (uint8_t i = 2u; i < 63u; i++) crc ^= p[i];
    p[63u] = crc;

    log_pend = 1u;
}

static float fgen_update(FOC_FuncGen_t *fg)
{
    fg->phase += fg->frequency * FOC_TS_HW;
    if (fg->phase >= 1.0f) fg->phase -= 1.0f;

    float out;
    switch (fg->waveform) {
        case 1u: /* square */
            out = (fg->phase < 0.5f) ? fg->amplitude : -fg->amplitude;
            break;
        case 2u: /* triangle */
            out = fg->amplitude * ((fg->phase < 0.5f)
                ? (4.0f * fg->phase - 1.0f)
                : (3.0f - 4.0f * fg->phase));
            break;
        case 3u: { /* sine — uses LUT, no math.h */
            float s, c;
            FOC_Math_SinCos(fg->phase * FOC_TWO_PI, &s, &c);
            out = fg->amplitude * s;
            break;
        }
        case 4u: {
            /* 2-step staircase: 0 -> A -> 2A, repeating every period.
             * The A->2A leg never crosses zero current, so the dead-time
             * phase-current-sign disturbance stays fixed across that edge —
             * that's the "second step" meant for clean R/L identification;
             * discard the 0->A and 2A->0 legs, which do cross zero. */
            const float third = 1.0f / 3.0f;
            if (fg->phase < third)             out = 0.0f;
            else if (fg->phase < 2.0f * third) out = fg->amplitude;
            else                                out = 2.0f * fg->amplitude;
            break;
        }
        case 0u: /* step — constant amplitude */
        default:
            out = fg->amplitude;
            break;
    }
    return out + fg->offset;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t log_tick = 0u;

    if (htim->Instance != TIM1) return;
    if (isr_mode == ISR_MODE_ADC_CAL) return;

    if (cmd_ready) {
        cmd_ready = 0u;
        cmd_apply(pending_cmd, pending_val);
    }

    uint32_t t0 = DWT->CYCCNT;

    motor.state.theta_mech_raw = (float)(as5147_rx & AS5147_RESP_DATA_MASK)
                                 * (2.0f * 3.14159265f / 16384.0f);
    HAL_GPIO_WritePin(AS5147_CSN_PORT, AS5147_CSN_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)&as5147_tx, (uint8_t*)&as5147_rx, 1);

    /* Apply function generator — voltage mode only; other modes commented out */
    if (fgen.enabled && !motor_stopped) {
        float fgen_out = fgen_update(&fgen);
        if (motor.ref.mode == FOC_MODE_VOLTAGE) {
//            motor.ref.v_q_ref = fgen_out;
        	motor.ref.v_d_ref = fgen_out; //Just for measuring d axis parameters
        }
//      else if (motor.ref.mode == FOC_MODE_TORQUE)    motor.ref.i_d_ref   = fgen_out; //For testing id current control, this should be iq
        else if (motor.ref.mode == FOC_MODE_TORQUE)    motor.ref.i_q_ref   = fgen_out;
//      else if (motor.ref.mode == FOC_MODE_VELOCITY)  motor.ref.omega_ref = fgen_out;
//      else if (motor.ref.mode == FOC_MODE_POSITION)  motor.ref.theta_ref = fgen_out;
    }

    FOC_Step(&motor);
//  PWM_SetDuties(motor.out.duty_u, motor.out.duty_v, motor.out.duty_w);  // V/W original order
    PWM_SetDuties(motor.out.duty_u, motor.out.duty_w, motor.out.duty_v);  // V and W swapped

//    uint32_t dac_val = (uint32_t)(motor.state.theta_mech_st * (4095.0f / (2.0f * 3.14159265f)));
//    if (dac_val > 4095u) dac_val = 4095u;
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
//    float e_norm = motor.state.theta_elec * (1.0f / (2.0f * 3.14159265f));
//    e_norm -= (float)(int32_t)e_norm;
//    if (e_norm < 0.0f) e_norm += 1.0f;
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(e_norm * 4095.0f));

//    /* i_u on DAC CH1: 0V = -10A, 1.65V = 0A, 3.3V = +10A */
//    float i_u_norm = motor.state.i_u * (1.0f / 10.0f);
//    if (i_u_norm >  1.0f) i_u_norm =  1.0f;
//    if (i_u_norm < -1.0f) i_u_norm = -1.0f;
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
//                     (uint32_t)((i_u_norm + 1.0f) * 0.5f * 4095.0f));
    /* i_u on DAC CH1: 0V = -10A, 1.65V = 0A, 3.3V = +10A */
//    float i_d_norm = motor.state.i_d * (1.0f / 1.0f);
//    if (i_d_norm >  1.0f) i_d_norm =  1.0f;
//    if (i_d_norm < -1.0f) i_d_norm = -1.0f;
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
//                     (uint32_t)((i_d_norm + 1.0f) * 0.5f * 4095.0f));

    float i_q_norm = motor.state.i_q * (1.0f / 1.0f);
    if (i_q_norm >  1.0f) i_q_norm =  1.0f;
    if (i_q_norm < -1.0f) i_q_norm = -1.0f;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                     (uint32_t)((i_q_norm + 1.0f) * 0.5f * 4095.0f));

    //For max 0.5
//    float i_d_norm = motor.state.i_d * (1.0f / 0.5f);
//    float i_d_norm = motor.state.i_d * (1.0f / 1.0f);
//    if (i_d_norm >  1.0f) i_d_norm =  1.0f;
//    if (i_d_norm < -1.0f) i_d_norm = -1.0f;
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
//                     (uint32_t)((i_d_norm + 1.0f) * 0.5f * 4095.0f));


//        float v_d_norm = motor.out.v_d* (1.0f / 0.4f);
//        if (v_d_norm >  1.0f) v_d_norm =  1.0f;
//        if (v_d_norm < -1.0f) v_d_norm = -1.0f;
//        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
//                         (uint32_t)((v_d_norm + 1.0f) * 0.5f * 4095.0f));
//    PWM_SetDuties(0.8, 0.5, 0.3);
    isr_cycles = DWT->CYCCNT - t0;

//    static uint32_t log_tick = 0u;
    if (++log_tick >= LOG_DECIMATE) {
        log_tick = 0u;
        log_pack();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2) return;
    HAL_GPIO_WritePin(AS5147_CSN_PORT, AS5147_CSN_PIN, GPIO_PIN_SET);
}

static void cmd_parse(void)
{
    /* Poll DMA write pointer and scan for valid 7-byte frames.
     * Frame: [0xBB][CMD][float32 LE, 4 B][XOR CRC]
     * Bytes are consumed here; command is posted to mailbox for ISR to apply. */
    uint8_t dma_wr = (uint8_t)(CMD_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    uint8_t avail  = (uint8_t)((dma_wr - cmd_dma_rd + CMD_DMA_BUF_SIZE) & (CMD_DMA_BUF_SIZE - 1u));

    while (avail >= 7u) {
        if (cmd_dma_buf[cmd_dma_rd] != 0xBBu) {
            cmd_dma_rd = (cmd_dma_rd + 1u) & (CMD_DMA_BUF_SIZE - 1u);
            avail--;
            continue;
        }
        uint8_t crc = 0u;
        for (uint8_t i = 1u; i < 6u; i++)
            crc ^= cmd_dma_buf[(cmd_dma_rd + i) & (CMD_DMA_BUF_SIZE - 1u)];
        if (crc != cmd_dma_buf[(cmd_dma_rd + 6u) & (CMD_DMA_BUF_SIZE - 1u)]) {
            cmd_dma_rd = (cmd_dma_rd + 1u) & (CMD_DMA_BUF_SIZE - 1u);
            avail--;
            continue;
        }
        uint8_t cmd = cmd_dma_buf[(cmd_dma_rd + 1u) & (CMD_DMA_BUF_SIZE - 1u)];
        uint8_t fb[4];
        for (uint8_t i = 0u; i < 4u; i++)
            fb[i] = cmd_dma_buf[(cmd_dma_rd + 2u + i) & (CMD_DMA_BUF_SIZE - 1u)];
        float val;
        memcpy(&val, fb, 4u);
        cmd_dma_rd = (cmd_dma_rd + 7u) & (CMD_DMA_BUF_SIZE - 1u);
        avail -= 7u;

        if (!cmd_ready) {
            pending_val = val;
            pending_cmd = cmd;
            cmd_ready   = 1u;
        }
        /* If cmd_ready is already set the ISR hasn't applied the previous command
         * yet — drop this frame. Commands arrive at human speed (ms intervals) so
         * this should never happen in practice. */
    }
}

static void cmd_apply(uint8_t cmd, float val)
{
    /* Called from ISR context — safe to write motor.ref.* directly. */
    switch (cmd) {
        case 0x01u: { /* SET_MODE */
            FOC_CtrlMode_t new_mode = (FOC_CtrlMode_t)val;
            /* While stopped, the live ref.mode must stay pinned at
             * FOC_MODE_VOLTAGE (gate disabled) — switching it to a PI-driven
             * mode here would run the current/velocity/position loop against
             * a bridge that can't respond, winding up the integrator before
             * RESUME ever re-enables it. Redirect the request to
             * mode_before_stop instead; RESUME already applies it. */
            FOC_CtrlMode_t *target = motor_stopped ? &mode_before_stop : &motor.ref.mode;
            if (new_mode != *target) {
                motor.ref.v_d_ref   = 0.0f;
                motor.ref.v_q_ref   = 0.0f;
                motor.ref.i_d_ref   = 0.0f;
                motor.ref.i_q_ref   = 0.0f;
                motor.ref.omega_ref = 0.0f;
                /* Entering position mode: hold the current position instead of
                 * snapping theta_ref to 0, which would command a step to the
                 * calibration-zero angle and jerk the rotor. */
                motor.ref.theta_ref = (new_mode == FOC_MODE_POSITION)
                                     ? (motor.state.theta_mech - motor.hw.theta_mech_offset)
                                     : 0.0f;
                fgen.enabled = 0u;
                *target      = new_mode;
                FOC_Reset();
            }
            break;
        }
        case 0x02u: motor.ref.v_d_ref = val; break;  /* SET_VD_REF */
        case 0x03u: motor.ref.v_q_ref = val; break;  /* SET_VQ_REF */
        case 0x04u: motor.ref.i_d_ref   = val; break;  /* SET_ID_REF  */
        case 0x05u: motor.ref.i_q_ref   = val; break;  /* SET_IQ_REF  */
//      case 0x06u: motor.ref.omega_ref  = val; break;  /* SET_OMEGA_REF — velocity disabled  */
//      case 0x07u: motor.ref.theta_ref  = val; break;  /* SET_THETA_REF — position disabled  */
        case 0x08u: { /* STOP */
            /* Drop to voltage mode at 0V instead of leaving the previous
             * mode running with zeroed refs: with the gate driver disabled
             * the inner loops would keep integrating against a measured
             * state they can no longer correct, winding up the PID(s) and
             * causing a jump the moment RESUME re-enables the bridge. */
            if (!motor_stopped) {
                mode_before_stop = motor.ref.mode;
            }
            motor.ref.v_d_ref   = 0.0f;
            motor.ref.v_q_ref   = 0.0f;
            motor.ref.i_d_ref   = 0.0f;
            motor.ref.i_q_ref   = 0.0f;
            motor.ref.omega_ref = 0.0f;
            motor.ref.theta_ref = 0.0f;
            motor.ref.mode      = FOC_MODE_VOLTAGE;
            fgen.enabled        = 0u;
            FOC_Reset();
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            motor_stopped = 1u;
            break;
        }
        case 0x09u: { /* RESUME */
            if (motor_stopped) {
                /* Position mode: hold the current position rather than
                 * resuming with the stale theta_ref zeroed out by STOP. */
                if (mode_before_stop == FOC_MODE_POSITION) {
                    motor.ref.theta_ref = motor.state.theta_mech - motor.hw.theta_mech_offset;
                }
                motor.ref.mode = mode_before_stop;
                FOC_Reset();
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
                motor_stopped = 0u;
            }
            break;
        }
        case 0x0Au: fgen.waveform  = (uint8_t)val;            break;  /* FGEN_WAVE   */
        case 0x0Bu: fgen.frequency = val;                      break;  /* FGEN_FREQ   */
        case 0x0Cu: fgen.amplitude = val;                      break;  /* FGEN_AMP    */
        case 0x0Du: fgen.offset    = val;                      break;  /* FGEN_OFFSET */
        case 0x0Eu: fgen.enabled   = (val != 0.0f) ? 1u : 0u; break;  /* FGEN_ENABLE */
        default: break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  HAL_Delay(6000);	//Test power-up delay

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting main loop \r\n");

  uint32_t t_stop = HAL_GetTick() + 80000U; /* 40 s safety limit */

  // Motor params
  motor.params.Rs         = FOC_MOTOR_RS;
  motor.params.Ld         = FOC_MOTOR_LD;
  motor.params.Lq         = FOC_MOTOR_LQ;
  motor.params.lambda_pm  = FOC_MOTOR_LAMBDA_PM;
  motor.params.pole_pairs = FOC_MOTOR_POLE_PAIRS;

  // HW config
  motor.hw.Ts               = FOC_TS_HW;
  motor.hw.duty_max         = FOC_DUTY_MAX;
  motor.hw.pwm_active_low   = FOC_PWM_ACTIVE_LOW;
  motor.hw.theta_elec_offset = 0.0f;
  motor.hw.theta_mech_offset = 0.0f;
  motor.hw.phase_reversed   = 0;
//  motor.hw.encoder_reversed = 1;  // OLD: negated theta_mech_st, causing negative omega_mech
  motor.hw.encoder_reversed = 0;
//  motor.hw.theta_elec_offset = -2.443; //From first oscilloscope comparison
//  motor.hw.theta_elec_offset = -2.663; //Newest, from 2nd oscilloscope comparison and MATLAB UV backEMF reconstruction


  // Bus voltage — fixed constant until ADC measurement is wired
  motor.state.v_bus = FOC_V_BUS_NOMINAL;

  // Refs held at zero — offset calibration runs first, motor stays still after
  motor.ref.v_d_ref = 0.0f;
  motor.ref.v_q_ref = 0.0f;
//  motor.ref.v_d_ref = 0.0f;
//  motor.ref.v_q_ref = 1.5f;
//  motor.ref.v_d_ref = 0.0f;
//  motor.ref.v_q_ref = 1.0f;

  FOC_Init();

  /* Current loop PI gains — pole placement: Kp = 2*zeta*wn*L - R,  Ki = L*wn^2 (cont.)
   * Ki is scaled by Ts here for forward-Euler discretisation (integrator += Ki*error each step). */
  {
      const float R     = motor.params.Rs;
      const float L     = motor.params.Ld;           /* Ld = Lq by assumption */
      const float Kp    = 2.0f * FOC_CURRENT_ZETA * FOC_CURRENT_WN * L - R;
      const float Ki    = L * (FOC_CURRENT_WN * FOC_CURRENT_WN) * FOC_TS_HW;
      const float v_lim = FOC_V_BUS_NOMINAL * FOC_ONE_OVER_SQRT3;
      FOC_PID_Init(&foc_pid_id, Kp, Ki, 0.0f, -v_lim, v_lim);
      FOC_PID_Init(&foc_pid_iq, Kp, Ki, 0.0f, -v_lim, v_lim);

      /* Same Kp/Ki -> identical poles; kept initialised so the I-P swap
       * in FOC_CurrentCtrlComputation (foc.c) works without touching this file. */
      FOC_IP_Init(&foc_ip_id, Kp, Ki, -v_lim, v_lim);
      FOC_IP_Init(&foc_ip_iq, Kp, Ki, -v_lim, v_lim);
  }

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;

  // INLx LOW (PC9) — keep low-side off during init
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  // Enable DRV8323 (PB13 HIGH) — required to power the CSA
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(1);

  // Configure DRV8323 via SPI — sets CSA gain and VREF_DIV before ADC cal
  FOC_DRV8323_Init(DRV8323_SPI_Transfer);
  printf("DRV 00=%03X 01=%03X 02=%03X 03=%03X 04=%03X 05=%03X 06=%03X\r\n",
         DRV8323_ReadReg(0x00), DRV8323_ReadReg(0x01),
         DRV8323_ReadReg(0x02), DRV8323_ReadReg(0x03),
         DRV8323_ReadReg(0x04), DRV8323_ReadReg(0x05),
         DRV8323_ReadReg(0x06));


  // Start PWM outputs — INLx=LOW prevents any switching; safe during cal
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //PA8  — phase W
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //PA9  — phase V
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //PA10 — phase U
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //ADC trigger

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 450);

  // --- ADC offset calibration -------------------------------------------
  // CSA active with correct gain/VREF_DIV. INLx=LOW guarantees zero current.
  HAL_TIM_Base_Start_IT(&htim1);
  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) { Error_Handler(); }
  if (HAL_ADCEx_InjectedStart(&hadc3) != HAL_OK) { Error_Handler(); }
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  while (!adc_cal_finished); // ~12.8 ms at 20 kHz, 256 samples

  printf("ADC cal done: zero_u=%lu zero_v=%lu zero_w=%lu\r\n",
         adc_zero[0], adc_zero[1], adc_zero[2]);

  // INLx HIGH then immediately switch ISR to CONTROL — no gap where FOC_Step
  // runs without switching enabled.
  FOC_Reset();
  FOC_Calibrate(&motor, FOC_CAL_V_D, FOC_CAL_SETTLE_TIME);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  isr_mode = ISR_MODE_CONTROL;
  HAL_UART_Receive_DMA(&huart1, cmd_dma_buf, CMD_DMA_BUF_SIZE);


//  HAL_Delay(1000);
  // INLx HIGH (PC9)
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

//  HAL_Delay(1000);

//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    cmd_parse();

    static uint8_t cal_printed = 0u;
    if (!cal_printed && motor.ref.mode == FOC_MODE_VOLTAGE) {
        printf("theta_elec_offset = %.4f rad\r\n", motor.hw.theta_elec_offset);
        cal_printed = 1u;
        /* v_q_ref left at 0 — PC scope sets references */
    }

    /* huart1.gState checks TX side only; HAL_UART_GetState() ORs in RxState
       (BUSY_RX when IT receive is armed) which would always block TX. */
    if (log_pend && (huart1.gState == HAL_UART_STATE_READY)) {
        uint8_t tx_idx = log_wr;
        log_wr   ^= 1u;
        log_pend  = 0u;
        HAL_UART_Transmit_DMA(&huart1, log_buf[tx_idx], LOG_FRAME_BYTES);
    }

//    if (HAL_GetTick() >= t_stop) {
//        motor.ref.v_q_ref = 0.0f;
//        motor.ref.v_d_ref = 0.0f;
//        fgen.enabled      = 0u;
//        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//        motor_stopped     = 1u;
//    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC4 PC6 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    GPIOC->ODR ^= GPIO_PIN_6;
    if (hadc->Instance != ADC1) return;

    uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    uint32_t raw_w = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);

    if (isr_mode == ISR_MODE_ADC_CAL) {
        if (!adc_cal_finished) {
            adc_cal_sum[0] += raw_u;
            adc_cal_sum[1] += raw_v;
            adc_cal_sum[2] += raw_w;
            if (++adc_cal_count >= ADC_CAL_SAMPLES) {
                adc_zero[0] = adc_cal_sum[0] / ADC_CAL_SAMPLES;
                adc_zero[1] = adc_cal_sum[1] / ADC_CAL_SAMPLES;
                adc_zero[2] = adc_cal_sum[2] / ADC_CAL_SAMPLES;
                adc_cal_finished = 1u;
            }
        }
        return;
    }

    const float scale = FOC_ADC_VREF / (float)(1u << FOC_ADC_BITS)
                        / (FOC_ADC_RSHUNT * FOC_ADC_CSA_GAIN_VV);

//  motor.state.i_u = ((float)raw_u - (float)adc_zero[0]) * scale;  // V/W original order
//  motor.state.i_v = ((float)raw_v - (float)adc_zero[1]) * scale;
//  motor.state.i_w = ((float)raw_w - (float)adc_zero[2]) * scale;
    motor.state.i_u = ((float)raw_u - (float)adc_zero[0]) * scale;
    motor.state.i_w = ((float)raw_v - (float)adc_zero[1]) * scale;  // ADC2 (HW phase V) → i_w
    motor.state.i_v = ((float)raw_w - (float)adc_zero[2]) * scale;  // ADC3 (HW phase W) → i_v
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

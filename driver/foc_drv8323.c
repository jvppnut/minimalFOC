#include "foc_drv8323.h"

/* --------------------------------------------------------------------------
 * SPI frame helpers
 *
 * 16-bit frame layout:
 *   Bit 15     : R/W (1 = read, 0 = write)
 *   Bits 14:11 : register address (4 bits)
 *   Bits 10:0  : data (11 bits)
 * -------------------------------------------------------------------------- */

static void drv_write(FOC_SPI_Transfer_t spi_xfer, uint8_t addr, uint16_t data)
{
    uint16_t frame = ((uint16_t)(addr & 0x0Fu) << 11u) | (data & 0x7FFu);
    uint8_t tx[2]  = { (uint8_t)(frame >> 8u), (uint8_t)(frame & 0xFFu) };
    uint8_t rx[2];
    spi_xfer(tx, rx, 2u);
}

static uint16_t drv_read(FOC_SPI_Transfer_t spi_xfer, uint8_t addr)
{
    uint16_t frame = (1u << 15u) | ((uint16_t)(addr & 0x0Fu) << 11u);
    uint8_t tx[2]  = { (uint8_t)(frame >> 8u), (uint8_t)(frame & 0xFFu) };
    uint8_t rx[2]  = { 0u, 0u };
    spi_xfer(tx, rx, 2u);
    return ((uint16_t)rx[0] << 8u | (uint16_t)rx[1]) & 0x7FFu;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void FOC_DRV8323_Init(FOC_SPI_Transfer_t spi_xfer)
{
    uint16_t reg;

    /* Driver Control (0x02) */
    reg  = (uint16_t)(FOC_DRV8323_DIS_CPUV  & 0x1u) << 10u;
    reg |= (uint16_t)(FOC_DRV8323_DIS_GDF   & 0x1u) << 9u;
    reg |= (uint16_t)(FOC_DRV8323_OTW_REP   & 0x1u) << 8u;
    reg |= (uint16_t)(FOC_DRV8323_PWM_MODE  & 0x3u) << 6u;
    drv_write(spi_xfer, DRV8323_REG_DRIVER_CTRL, reg);

    /* Gate Drive HS (0x03) */
    reg  = (uint16_t)(FOC_DRV8323_IDRIVEP_HS & 0xFu) << 4u;
    reg |= (uint16_t)(FOC_DRV8323_IDRIVEN_HS & 0xFu) << 0u;
    drv_write(spi_xfer, DRV8323_REG_GATE_DRIVE_HS, reg);

    /* Gate Drive LS (0x04) */
    reg  = (uint16_t)(FOC_DRV8323_TDRIVE     & 0x3u) << 8u;
    reg |= (uint16_t)(FOC_DRV8323_IDRIVEP_LS & 0xFu) << 4u;
    reg |= (uint16_t)(FOC_DRV8323_IDRIVEN_LS & 0xFu) << 0u;
    drv_write(spi_xfer, DRV8323_REG_GATE_DRIVE_LS, reg);

    /* OCP Control (0x05) */
    reg  = (uint16_t)(FOC_DRV8323_TRETRY    & 0x1u) << 9u;
    reg |= (uint16_t)(FOC_DRV8323_DEAD_TIME & 0x7u) << 6u;
    reg |= (uint16_t)(FOC_DRV8323_OCP_MODE  & 0x3u) << 4u;
    reg |= (uint16_t)(FOC_DRV8323_OCP_DEG   & 0x3u) << 2u;
    reg |= (uint16_t)(FOC_DRV8323_VDS_LVL   & 0x3u) << 0u;
    drv_write(spi_xfer, DRV8323_REG_OCP_CTRL, reg);

    /* CSA Control (0x06) */
    reg  = (uint16_t)(FOC_DRV8323_CSA_FET  & 0x1u) << 10u;
    reg |= (uint16_t)(FOC_DRV8323_VREF_DIV & 0x1u) << 9u;
    reg |= (uint16_t)(FOC_DRV8323_LS_REF   & 0x1u) << 8u;
    reg |= (uint16_t)(FOC_DRV8323_CSA_GAIN & 0x3u) << 6u;
    reg |= (uint16_t)(FOC_DRV8323_DIS_SEN  & 0x1u) << 5u;
    reg |= (uint16_t)(FOC_DRV8323_SEN_LVL  & 0x7u) << 2u;
    drv_write(spi_xfer, DRV8323_REG_CSA_CTRL, reg);
}

void FOC_DRV8323_ClearFaults(FOC_SPI_Transfer_t spi_xfer)
{
    uint16_t reg = drv_read(spi_xfer, DRV8323_REG_DRIVER_CTRL);
    drv_write(spi_xfer, DRV8323_REG_DRIVER_CTRL, reg | (1u << 1u));
}

#ifndef FOC_AS5147_H
#define FOC_AS5147_H

#include <stdint.h>

/* ==========================================================================
 * AS5147 — 14-bit magnetic rotary position sensor.
 *
 * SPI frame (16-bit, CPOL=0 CPHA=1, MSB first):
 *   Bit 15     : even parity over bits [14:0]
 *   Bit 14     : R/W  (1 = read, 0 = write)
 *   Bits 13:0  : register address
 *
 * Response frame (pipelined — arrives one transaction after the command):
 *   Bit 15     : even parity
 *   Bit 14     : EF  (error flag; check ERRFL if set)
 *   Bits 13:0  : 14-bit data
 *
 * Angle output: 14-bit unsigned, LSB = 360° / 16384 ≈ 0.022°/count.
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * Register addresses
 * -------------------------------------------------------------------------- */
#define AS5147_REG_NOP       0x0000u
#define AS5147_REG_ERRFL     0x0001u
#define AS5147_REG_PROG      0x0003u
#define AS5147_REG_DIAAGC    0x3FFCu
#define AS5147_REG_MAG       0x3FFDu
#define AS5147_REG_ANGLEUNC  0x3FFEu  /* angle without dynamic error compensation */
#define AS5147_REG_ANGLECOM  0x3FFFu  /* angle with dynamic error compensation — use for FOC */

/* --------------------------------------------------------------------------
 * Response data mask
 * -------------------------------------------------------------------------- */
#define AS5147_RESP_DATA_MASK  0x3FFFu  /* 14-bit angle/data payload */
#define AS5147_RESP_EF         (1u << 14)

/* --------------------------------------------------------------------------
 * DMA read command for ANGLECOM
 *
 * Load this word into the STM32 DMA TX buffer. The RX buffer receives the
 * angle from the previous cycle (pipelined).
 *
 * Derivation: R/W=1, addr=0x3FFF
 *   bits[14:0] = 0x7FFF (15 ones, odd count) → parity = 1 → frame = 0xFFFF
 * -------------------------------------------------------------------------- */
#define AS5147_CMD_READ_ANGLECOM  0xFFFFu

#endif /* FOC_AS5147_H */

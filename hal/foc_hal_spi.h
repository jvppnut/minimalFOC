#ifndef FOC_HAL_SPI_H
#define FOC_HAL_SPI_H

#include <stdint.h>

/* Generic SPI transfer function pointer used by all peripheral drivers.
 *
 * tx  : bytes to transmit, MSB first
 * rx  : bytes received,    MSB first; may be NULL if receive unused
 * len : number of bytes in the transaction
 *
 * CS assertion/deassertion is the responsibility of the platform
 * implementation — the driver does not touch CS directly. */
typedef void (*FOC_SPI_Transfer_t)(const uint8_t *tx, uint8_t *rx, uint8_t len);

#endif /* FOC_HAL_SPI_H */

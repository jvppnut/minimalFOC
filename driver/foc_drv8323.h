#ifndef FOC_DRV8323_H
#define FOC_DRV8323_H

#include <stdint.h>
#include "hal/foc_hal_spi.h"
#include "foc_config.h"

/* ==========================================================================
 * DRV8323R — 3-phase smart gate driver with integrated current shunt amps.
 *
 * SPI frame (16-bit, CPOL=0 CPHA=1 — verify mode against datasheet):
 *   Bit 15     : R/W  (1 = read, 0 = write)
 *   Bits 14:11 : register address (4 bits)
 *   Bits 10:0  : data (11 bits); send zeros on MOSI for reads
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * Register addresses
 * -------------------------------------------------------------------------- */
#define DRV8323_REG_FAULT_STATUS1   0x00u
#define DRV8323_REG_VGS_STATUS2     0x01u
#define DRV8323_REG_DRIVER_CTRL     0x02u
#define DRV8323_REG_GATE_DRIVE_HS   0x03u
#define DRV8323_REG_GATE_DRIVE_LS   0x04u
#define DRV8323_REG_OCP_CTRL        0x05u
#define DRV8323_REG_CSA_CTRL        0x06u

/* --------------------------------------------------------------------------
 * Fault Status 1 (0x00) bit masks                          verify bit positions
 * -------------------------------------------------------------------------- */
#define DRV8323_FS1_FAULT       (1u << 10)
#define DRV8323_FS1_VDS_OCP     (1u << 9)
#define DRV8323_FS1_GDF         (1u << 8)
#define DRV8323_FS1_UVLO        (1u << 7)
#define DRV8323_FS1_OTSD        (1u << 6)
#define DRV8323_FS1_VDS_HA      (1u << 5)
#define DRV8323_FS1_VDS_LA      (1u << 4)
#define DRV8323_FS1_VDS_HB      (1u << 3)
#define DRV8323_FS1_VDS_LB      (1u << 2)
#define DRV8323_FS1_VDS_HC      (1u << 1)
#define DRV8323_FS1_VDS_LC      (1u << 0)

/* --------------------------------------------------------------------------
 * VGS Status 2 (0x01) bit masks                            verify bit positions
 * -------------------------------------------------------------------------- */
#define DRV8323_FS2_SA_OC       (1u << 10)
#define DRV8323_FS2_SB_OC       (1u << 9)
#define DRV8323_FS2_SC_OC       (1u << 8)
#define DRV8323_FS2_OTW         (1u << 7)
#define DRV8323_FS2_GDUV        (1u << 6)
#define DRV8323_FS2_VGS_HA      (1u << 5)
#define DRV8323_FS2_VGS_LA      (1u << 4)
#define DRV8323_FS2_VGS_HB      (1u << 3)
#define DRV8323_FS2_VGS_LB      (1u << 2)
#define DRV8323_FS2_VGS_HC      (1u << 1)
#define DRV8323_FS2_VGS_LC      (1u << 0)

/* --------------------------------------------------------------------------
 * PWM mode field values — Driver Control bits [7:6]         verify
 * -------------------------------------------------------------------------- */
#define DRV8323_PWM_6X          0u
#define DRV8323_PWM_3X          1u
#define DRV8323_PWM_1X          2u
#define DRV8323_PWM_INDEPENDENT 3u

/* --------------------------------------------------------------------------
 * CSA gain field values — CSA Control bits [7:6]            verify
 * -------------------------------------------------------------------------- */
#define DRV8323_CSA_GAIN_5X     0u
#define DRV8323_CSA_GAIN_10X    1u
#define DRV8323_CSA_GAIN_20X    2u
#define DRV8323_CSA_GAIN_40X    3u

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/* Configure all writable registers from the FOC_DRV8323_* macros in foc_config.h. */
void FOC_DRV8323_Init(FOC_SPI_Transfer_t spi_xfer);

/* Set CLR_FLT via read-modify-write on Driver Control to clear latched faults. */
void FOC_DRV8323_ClearFaults(FOC_SPI_Transfer_t spi_xfer);

#endif /* FOC_DRV8323_H */

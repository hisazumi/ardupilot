/*******************************************************************************
 * ArduPilot Platform User Defines for VL53L3C
 *
 * Based on STMicroelectronics VL53LX platform user defines
 *
 * This file contains platform-specific macro definitions for the VL53L3C
 * bare driver when used with ArduPilot.
 ******************************************************************************/

#ifndef _VL53LX_PLATFORM_USER_DEFINES_H_
#define _VL53LX_PLATFORM_USER_DEFINES_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Division macro for unsigned integers
 */
#define do_division_u(dividend, divisor) ((dividend) / (divisor))

/**
 * @brief Division macro for signed integers
 */
#define do_division_s(dividend, divisor) ((dividend) / (divisor))

/**
 * @brief Warning override status macro
 */
#define WARN_OVERRIDE_STATUS(__X__)

/**
 * @brief Disable compiler warnings (no-op for ArduPilot)
 */
#define DISABLE_WARNINGS()

/**
 * @brief Enable compiler warnings (no-op for ArduPilot)
 */
#define ENABLE_WARNINGS()

#ifdef __cplusplus
}
#endif

#endif  /* _VL53LX_PLATFORM_USER_DEFINES_H_ */

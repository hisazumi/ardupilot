/*******************************************************************************
 * ArduPilot Platform Logging for VL53L3C
 *
 * Based on STMicroelectronics VL53LX platform log
 *
 * This file provides logging macros for the VL53L3C bare driver.
 * Logging is disabled by default for ArduPilot to minimize overhead.
 ******************************************************************************/

#ifndef _VL53LX_PLATFORM_LOG_H_
#define _VL53LX_PLATFORM_LOG_H_

#include "vl53lx_platform_user_config.h"
#include "vl53lx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Trace level definitions
#define VL53LX_TRACE_LEVEL_NONE            0x00000000
#define VL53LX_TRACE_LEVEL_ERRORS          0x00000001
#define VL53LX_TRACE_LEVEL_WARNING         0x00000002
#define VL53LX_TRACE_LEVEL_INFO            0x00000004
#define VL53LX_TRACE_LEVEL_DEBUG           0x00000008
#define VL53LX_TRACE_LEVEL_ALL             0x00000010
#define VL53LX_TRACE_LEVEL_IGNORE          0x00000020

// Trace function definitions
#define VL53LX_TRACE_FUNCTION_NONE         0x00000000
#define VL53LX_TRACE_FUNCTION_I2C          0x00000001
#define VL53LX_TRACE_FUNCTION_ALL          0x7fffffff

// Trace module definitions
#define VL53LX_TRACE_MODULE_NONE           0x00000000
#define VL53LX_TRACE_MODULE_API            0x00000001
#define VL53LX_TRACE_MODULE_CORE           0x00000002
#define VL53LX_TRACE_MODULE_PROTECTED      0x00000004
#define VL53LX_TRACE_MODULE_HISTOGRAM      0x00000008
#define VL53LX_TRACE_MODULE_REGISTERS      0x00000010
#define VL53LX_TRACE_MODULE_PLATFORM       0x00000020
#define VL53LX_TRACE_MODULE_NVM            0x00000040
#define VL53LX_TRACE_MODULE_CALIBRATION_DATA    0x00000080
#define VL53LX_TRACE_MODULE_NVM_DATA       0x00000100
#define VL53LX_TRACE_MODULE_HISTOGRAM_DATA 0x00000200
#define VL53LX_TRACE_MODULE_RANGE_RESULTS_DATA  0x00000400
#define VL53LX_TRACE_MODULE_XTALK_DATA     0x00000800
#define VL53LX_TRACE_MODULE_OFFSET_DATA    0x00001000
#define VL53LX_TRACE_MODULE_DATA_INIT      0x00002000
#define VL53LX_TRACE_MODULE_REF_SPAD_CHAR  0x00004000
#define VL53LX_TRACE_MODULE_SPAD_RATE_MAP  0x00008000
#define VL53LX_TRACE_MODULE_SPAD           0x01000000
#define VL53LX_TRACE_MODULE_FMT            0x02000000
#define VL53LX_TRACE_MODULE_UTILS          0x04000000
#define VL53LX_TRACE_MODULE_CUSTOMER_API   0x40000000
#define VL53LX_TRACE_MODULE_ALL            0x7fffffff

// Logging is disabled for ArduPilot - all logging macros are no-ops
#define VL53LX_trace_config(filename, modules, level, functions)
#define VL53LX_trace_print(module, level, function, ...)
#define trace_print(level, ...)

// Underlying logging macros used by bare driver files (all disabled)
#define _LOG_FUNCTION_START(module, fmt, ...)
#define _LOG_FUNCTION_END(module, status, ...)
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...)
#define _LOG_TRACE_PRINT(module, level, function, ...)
#define trace_print_module_function(module, level, function, ...)

// Function entry/exit logging macros (disabled)
#define LOG_FUNCTION_START(fmt, ...)
#define LOG_FUNCTION_END(status)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

// I2C logging macros (disabled)
#define VL53LX_trace_i2c(...)

#ifdef __cplusplus
}
#endif

#endif // _VL53LX_PLATFORM_LOG_H_

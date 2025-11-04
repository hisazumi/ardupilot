/*******************************************************************************
 * ArduPilot Platform IPP Imports for VL53L3C
 *
 * Based on STMicroelectronics VL53LX platform IPP imports
 *
 * Intel IPP (Integrated Performance Primitives) is not used in ArduPilot.
 * This file is a stub for compatibility.
 ******************************************************************************/

#ifndef _VL53LX_PLATFORM_IPP_IMPORTS_H_
#define _VL53LX_PLATFORM_IPP_IMPORTS_H_

// IPP is not used in ArduPilot - this file is a stub for compatibility
// The bare driver may conditionally include this file

#ifdef VL53LX_NEEDS_IPP
#  undef VL53LX_IPP_API
#  define VL53LX_IPP_API
#endif

#endif // _VL53LX_PLATFORM_IPP_IMPORTS_H_

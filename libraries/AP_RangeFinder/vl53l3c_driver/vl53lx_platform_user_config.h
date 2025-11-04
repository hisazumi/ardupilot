/*******************************************************************************
 * ArduPilot Platform User Configuration for VL53L3C
 *
 * Based on STMicroelectronics VL53LX platform user config
 *
 * This file contains compile-time configuration parameters for the VL53L3C
 * bare driver when used with ArduPilot.
 ******************************************************************************/

#ifndef _VL53LX_PLATFORM_USER_CONFIG_H_
#define _VL53LX_PLATFORM_USER_CONFIG_H_

// Byte alignment for multi-byte values
#define VL53LX_BYTES_PER_WORD              2
#define VL53LX_BYTES_PER_DWORD             4

// Polling delays (milliseconds)
#define VL53LX_BOOT_COMPLETION_POLLING_TIMEOUT_MS     500
#define VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS   2000
#define VL53LX_TEST_COMPLETION_POLLING_TIMEOUT_MS   10000

#define VL53LX_POLLING_DELAY_MS                         1

// Tuning parameter page base addresses
#define VL53LX_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS  0x8000
#define VL53LX_TUNINGPARM_PRIVATE_PAGE_BASE_ADDRESS 0xC000

// Calibration limits
#define VL53LX_OFFSET_CAL_MIN_MM1_EFFECTIVE_SPADS  0x0500
    /*!< Lower Limit for the MM1 effective SPAD count during offset
         calibration Format 8.8 0x0500 -> 5.0 effective SPADs */

#define VL53LX_GAIN_FACTOR__STANDARD_DEFAULT       0x0800
    /*!< Default standard ranging gain correction factor
         1.11 format. 1.0 = 0x0800, 0.980 = 0x07D7 */

#define VL53LX_GAIN_FACTOR__HISTOGRAM_DEFAULT      0x0800
    /*!< Default histogram ranging gain correction factor
         1.11 format. 1.0 = 0x0800, 0.975 = 0x07CC */

#define VL53LX_OFFSET_CAL_MIN_EFFECTIVE_SPADS  0x0500
    /*!< Lower Limit for the MM1 effective SPAD count during offset
         calibration Format 8.8 0x0500 -> 5.0 effective SPADs */

#define VL53LX_OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS   0x1900
    /*!< Max Limit for the pre range peak rate during offset
         calibration Format 9.7 0x1900 -> 50.0 Mcps.
         If larger then in pile up */

#define VL53LX_OFFSET_CAL_MAX_SIGMA_MM             0x0040
    /*!< Max sigma estimate limit during offset calibration
         Check applies to pre-range, mm1 and mm2 ranges
         Format 14.2 0x0040 -> 16.0mm. */

#define VL53LX_ZONE_CAL_MAX_PRE_PEAK_RATE_MCPS     0x1900
    /*!< Max Peak Rate Limit for the during zone calibration
         Format 9.7 0x1900 -> 50.0 Mcps.
         If larger then in pile up */

#define VL53LX_ZONE_CAL_MAX_SIGMA_MM               0x0040
    /*!< Max sigma estimate limit during zone calibration
         Format 14.2 0x0040 -> 16.0mm. */

#define VL53LX_XTALK_EXTRACT_MAX_SIGMA_MM          0x008C
    /*!< Max Sigma value allowed for a successful xtalk extraction
         Format 14.2 0x008C -> 35.0 mm.*/

#ifndef VL53LX_MAX_USER_ZONES
#define VL53LX_MAX_USER_ZONES                16
    /*!< Max number of user Zones - maximal limitation from
         FW stream divide - value of 254 */
#endif

#define VL53LX_MAX_RANGE_RESULTS              4
#define VL53LX_BUFFER_SIZE              5

    /*!< Sets the maximum number of targets distances the histogram
         post processing can generate */

#define VL53LX_MAX_STRING_LENGTH 512
    /*!< Sets the maximum string length */

#endif  /* _VL53LX_PLATFORM_USER_CONFIG_H_ */

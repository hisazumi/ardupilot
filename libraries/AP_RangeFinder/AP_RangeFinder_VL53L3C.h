#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L3C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_VL53L3C : public AP_RangeFinder_Backend
{

public:
    // VL53L3C supports Medium and Long distance modes (Histogram mode only)
    enum class DistanceMode { Medium, Long, Unknown };

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, DistanceMode mode);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // Device error codes - same as VL53L1X
    enum DeviceError : uint8_t
    {
        NOUPDATE                    = 0,
        VCSELCONTINUITYTESTFAILURE  = 1,
        VCSELWATCHDOGTESTFAILURE    = 2,
        NOVHVVALUEFOUND             = 3,
        MSRCNOTARGET                = 4,
        RANGEPHASECHECK             = 5,
        SIGMATHRESHOLDCHECK         = 6,
        PHASECONSISTENCY            = 7,
        MINCLIP                     = 8,
        RANGECOMPLETE               = 9,
        ALGOUNDERFLOW               = 10,
        ALGOOVERFLOW                = 11,
        RANGEIGNORETHRESHOLD        = 12,
        USERROICLIP                 = 13,
        REFSPADCHARNOTENOUGHDPADS   = 14,
        REFSPADCHARMORETHANTARGET   = 15,
        REFSPADCHARLESSTHANTARGET   = 16,
        MULTCLIPFAIL                = 17,
        GPHSTREAMCOUNT0READY        = 18,
        RANGECOMPLETE_NO_WRAP_CHECK = 19,
        EVENTCONSISTENCY            = 20,
        MINSIGNALEVENTCHECK         = 21,
        RANGECOMPLETE_MERGED_PULSE  = 22,
    };

    // VL53L3C Product ID
    static constexpr uint16_t VL53L3C_PRODUCT_ID = 0xEAAA;

    // Timing budget and measurement period
    static constexpr uint32_t MEASUREMENT_TIME_MS = 50;  // 20Hz update rate
    static constexpr uint32_t TIMING_BUDGET_US = 33000;  // 33ms (same as StampFly)

    // Register addresses (same as VL53L1X)
    enum regAddr : uint16_t
    {
        SOFT_RESET                                                                 = 0x0000,
        GPIO__TIO_HV_STATUS                                                        = 0x0031,
        RANGE_CONFIG__VCSEL_PERIOD_A                                              = 0x0060,
        RANGE_CONFIG__VCSEL_PERIOD_B                                              = 0x0063,
        RANGE_CONFIG__VALID_PHASE_LOW                                             = 0x0068,
        RANGE_CONFIG__VALID_PHASE_HIGH                                            = 0x0069,
        SD_CONFIG__WOI_SD0                                                         = 0x0078,
        SD_CONFIG__WOI_SD1                                                         = 0x0079,
        SD_CONFIG__INITIAL_PHASE_SD0                                              = 0x007A,
        SD_CONFIG__INITIAL_PHASE_SD1                                              = 0x007B,
        SYSTEM__INTERRUPT_CLEAR                                                    = 0x0086,
        SYSTEM__MODE_START                                                         = 0x0087,
        RESULT__RANGE_STATUS                                                       = 0x0089,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                            = 0x0096,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI                         = 0x0096,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO                         = 0x0097,
        PAD_I2C_HV__EXTSUP_CONFIG                                                  = 0x002E,
        VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                                      = 0x0008,
        PHASECAL_CONFIG__TIMEOUT_MACROP                                            = 0x004B,
        MM_CONFIG__TIMEOUT_MACROP_A                                                = 0x005A,
        MM_CONFIG__TIMEOUT_MACROP_A_HI                                             = 0x005A,
        MM_CONFIG__TIMEOUT_MACROP_A_LO                                             = 0x005B,
        MM_CONFIG__TIMEOUT_MACROP_B                                                = 0x005C,
        MM_CONFIG__TIMEOUT_MACROP_B_HI                                             = 0x005C,
        MM_CONFIG__TIMEOUT_MACROP_B_LO                                             = 0x005D,
        RANGE_CONFIG__TIMEOUT_MACROP_A                                             = 0x005E,
        RANGE_CONFIG__TIMEOUT_MACROP_A_HI                                          = 0x005E,
        RANGE_CONFIG__TIMEOUT_MACROP_A_LO                                          = 0x005F,
        RANGE_CONFIG__TIMEOUT_MACROP_B                                             = 0x0061,
        RANGE_CONFIG__TIMEOUT_MACROP_B_HI                                          = 0x0061,
        RANGE_CONFIG__TIMEOUT_MACROP_B_LO                                          = 0x0062,
        OSC_MEASURED__FAST_OSC__FREQUENCY                                          = 0x0006,
        OSC_MEASURED__FAST_OSC__FREQUENCY_HI                                       = 0x0006,
        OSC_MEASURED__FAST_OSC__FREQUENCY_LO                                       = 0x0007,
        MM_CONFIG__OUTER_OFFSET_MM                                                 = 0x0022,
        MM_CONFIG__OUTER_OFFSET_MM_HI                                              = 0x0022,
        MM_CONFIG__OUTER_OFFSET_MM_LO                                              = 0x0023,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM                                         = 0x001E,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM_HI                                      = 0x001E,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM_LO                                      = 0x001F,
        SYSTEM__INTERMEASUREMENT_PERIOD                                            = 0x006C,
        SYSTEM__INTERMEASUREMENT_PERIOD_B1                                         = 0x006C,
        SYSTEM__INTERMEASUREMENT_PERIOD_B2                                         = 0x006D,
        SYSTEM__INTERMEASUREMENT_PERIOD_B3                                         = 0x006E,
        SYSTEM__INTERMEASUREMENT_PERIOD_B4                                         = 0x006F,
        RESULT__OSC_CALIBRATE_VAL                                                  = 0x00DE,
        VHV_CONFIG__INIT                                                           = 0x00D1,
        PHASECAL_CONFIG__OVERRIDE                                                  = 0x00DD,
        PHASECAL_RESULT__VCSEL_START                                               = 0x00D8,
        CAL_CONFIG__VCSEL_START                                                    = 0x00D5,
    };

    // Constructor
    AP_RangeFinder_VL53L3C(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev);

    // Initialization and configuration
    bool init(DistanceMode mode) WARN_IF_UNUSED;
    bool check_id(void);
    bool reset(void) WARN_IF_UNUSED;
    bool setDistanceMode(DistanceMode distance_mode) WARN_IF_UNUSED;
    bool setMeasurementTimingBudget(uint32_t budget_us) WARN_IF_UNUSED;
    bool getMeasurementTimingBudget(uint32_t &budget) WARN_IF_UNUSED;
    bool startContinuous(uint32_t period_ms) WARN_IF_UNUSED;
    bool setupManualCalibration(void) WARN_IF_UNUSED;

    // Measurement functions
    void timer();
    bool get_reading(uint16_t &reading_mm);
    bool dataReady(void);

    // I2C register access
    bool read_register(uint16_t reg, uint8_t &value) WARN_IF_UNUSED;
    bool read_register16(uint16_t reg, uint16_t &value) WARN_IF_UNUSED;
    bool write_register(uint16_t reg, uint8_t value) WARN_IF_UNUSED;
    bool write_register16(uint16_t reg, uint16_t value) WARN_IF_UNUSED;
    bool write_register32(uint16_t reg, uint32_t value) WARN_IF_UNUSED;

    // Timeout and timing calculations
    uint32_t decodeTimeout(uint16_t reg_val);
    uint16_t encodeTimeout(uint32_t timeout_mclks);
    uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
    uint32_t calcMacroPeriod(uint8_t vcsel_period) const;

    // Member variables
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    uint32_t sum_mm;
    uint32_t counter;
    DistanceMode mode;

    // Calibration state
    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;
    uint16_t mm_config_outer_offset_mm;
    bool calibrated;

    // Timing state
    static constexpr uint32_t TimingGuard = 4528;
};

#endif  // AP_RANGEFINDER_VL53L3C_ENABLED

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST VL53L3CX lidar (Histogram mode)

  Based on VL53L1X driver and StampFly implementation
  VL53L3C uses Histogram mode exclusively with different VCSEL periods
 */
#include "AP_RangeFinder_VL53L3C.h"

#if AP_RANGEFINDER_VL53L3C_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_VL53L3C::AP_RangeFinder_VL53L3C(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev))
    , calibrated(false) {}

/*
   detect if a VL53L3C rangefinder is connected.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L3C::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, DistanceMode mode)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L3C *sensor
        = NEW_NOTHROW AP_RangeFinder_VL53L3C(_state, _params, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    if (!sensor->check_id() || !sensor->init(mode)) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->give();

    return sensor;
}

// check sensor ID matches VL53L3C
bool AP_RangeFinder_VL53L3C::check_id(void)
{
    uint8_t v1, v2;
    if (!(read_register(0x010F, v1) && read_register(0x0110, v2))) {
        return false;
    }

    // VL53L3C Product ID: 0xEAAA
    if (v1 == 0xEA && v2 == 0xAA) {
        printf("VL53L3C: Detected on bus 0x%02x\n", unsigned(dev->get_bus_id()));
        return true;
    }

    printf("VL53L3C: Unknown sensor ID: 0x%02X%02X\n", v1, v2);
    return false;
}

bool AP_RangeFinder_VL53L3C::reset(void)
{
    if (dev->get_bus_id() != 0x29) {
        // if sensor is on a different port than the default do not reset sensor otherwise we will lose the address.
        // we assume it is already configured.
        return true;
    }
    if (!write_register(SOFT_RESET, 0x00)) {
        return false;
    }
    hal.scheduler->delay_microseconds(100);
    if (!write_register(SOFT_RESET, 0x01)) {
        return false;
    }
    hal.scheduler->delay(1000);
    return true;
}

/*
  initialise sensor - based on StampFly VL53L3C implementation
 */
bool AP_RangeFinder_VL53L3C::init(DistanceMode distance_mode)
{
    // we need to do resets and delays in order to configure the sensor, don't do this if we are trying to fast boot
    if (hal.util->was_watchdog_armed()) {
        return false;
    }

    // Store distance mode
    mode = distance_mode;

    uint8_t pad_i2c_hv_extsup_config = 0;
    if (!(reset() &&
          // setup for 2.8V operation
          read_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config) &&
          write_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config | 0x01) &&

          // store oscillator info for later use
          read_register16(OSC_MEASURED__FAST_OSC__FREQUENCY, fast_osc_frequency) &&
          read_register16(RESULT__OSC_CALIBRATE_VAL, osc_calibrate_val))) {
              printf("VL53L3C: init sequence failed\n");
              return false;
          }

    // Debug: Show raw register bytes
    uint8_t osc_hi = 0, osc_lo = 0;
    if (read_register(OSC_MEASURED__FAST_OSC__FREQUENCY_HI, osc_hi) &&
        read_register(OSC_MEASURED__FAST_OSC__FREQUENCY_LO, osc_lo)) {
        printf("VL53L3C: OSC register bytes: 0x%02X%02X (hi=0x%02X, lo=0x%02X) = %u\n",
               osc_hi, osc_lo, osc_hi, osc_lo, fast_osc_frequency);
    }

    // Calculate and show PLL period
    uint32_t pll_period = ((uint32_t)0x01 << 30) / fast_osc_frequency;
    printf("VL53L3C: pll_period = (2^30) / %u = %lu\n",
           fast_osc_frequency, (unsigned long)pll_period);

    printf("VL53L3C: osc_calibrate_val=%u\n", osc_calibrate_val);

    if (!(// Set distance mode with VL53L3C-specific VCSEL periods
          setDistanceMode(mode) &&

          // Set timing budget (33ms as per StampFly)
          setMeasurementTimingBudget(TIMING_BUDGET_US) &&

          // Read calibration offset
          read_register16(MM_CONFIG__OUTER_OFFSET_MM, mm_config_outer_offset_mm) &&
          write_register16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, mm_config_outer_offset_mm * 4) &&

          // Set continuous mode
          startContinuous(MEASUREMENT_TIME_MS))) {
              printf("VL53L3C: init sequence failed (after osc read)\n");
              return false;
          }

    printf("VL53L3C: init successful, mode=%s\n",
           mode == DistanceMode::Medium ? "MEDIUM" : "LONG");

    // call timer() every MEASUREMENT_TIME_MS
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L3C::timer, void));

    return true;
}

// Set distance mode - VL53L3C uses different VCSEL periods than VL53L1X
bool AP_RangeFinder_VL53L3C::setDistanceMode(DistanceMode distance_mode)
{
    switch (distance_mode) {
      case DistanceMode::Medium:
            // VL53L3C MEDIUM mode - Complete configuration from VL53LX bare driver
            // vl53lx_api_preset_modes.c lines 1257-1291 (preset_mode_histogram_medium_range)
            // VCSEL periods: A=6, B=8 (different from VL53L1X: A=12, B=10)
            // Histogram bins for MEDIUM: (7,0,1,1,2,2) even, (0,1,2,1,2,3) odd
            // This generates VALID_PHASE via copy_hist_cfg_to_static_cfg():
            //   low_amb_odd_bin_2_3 = (1<<4)+2 = 0x12
            //   low_amb_odd_bin_4_5 = (3<<4)+2 = 0x32
            // VL53L3C MEDIUM mode configuration - write mode-specific registers only
            // Timeout macrop values will be calculated by setMeasurementTimingBudget()
            if (!(write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x05) &&  // 6 periods
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x07) &&  // 8 periods

                  // Calibration configuration (lines 1273, 1277)
                  // Note: PHASECAL_CONFIG__TIMEOUT_MACROP will be calculated in setMeasurementTimingBudget()
                  write_register(CAL_CONFIG__VCSEL_START, 0x05) &&

                  // Valid phase from histogram bins (after copy_hist_cfg_to_static_cfg)
                  write_register(RANGE_CONFIG__VALID_PHASE_LOW, 0x12) &&  // low_amb_odd_bin_2_3
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x32) &&  // low_amb_odd_bin_4_5

                  // Dynamic config (lines 1281-1286)
                  write_register(SD_CONFIG__WOI_SD0, 0x05) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x07) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 5) &&  // tp_init_phase_rtn_hist_med
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 6))) {  // tp_init_phase_ref_hist_med
                return false;
            }
            break;

        case DistanceMode::Long:
            // VL53L3C LONG mode - from VL53LX bare driver vl53lx_api_preset_modes.c
            // VCSEL periods: A=10, B=12 (vs VL53L1X: A=16, B=14)
            // INITIAL_PHASE: SD0=RTN(9), SD1=REF(6)
            // VALID_PHASE (timing_config): Derived from histogram bins (0,1,2,3,4,5)
            // - low_amb_odd_bin_2_3 = (3<<4)+2 = 0x32
            // - low_amb_odd_bin_4_5 = (5<<4)+4 = 0x54
            if (!(write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x09) &&  // 10 periods
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0B) &&  // 12 periods
                  write_register(RANGE_CONFIG__VALID_PHASE_LOW, 0x32) &&  // from histogram low_amb_odd_bin_2_3
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x54) &&  // from histogram low_amb_odd_bin_4_5

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x09) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x0B) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 9) &&  // RTN phase (tp_init_phase_rtn_hist_long)
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 6))) {  // REF phase (tp_init_phase_ref_hist_long)
                return false;
            }
            break;

        default:
            // unrecognized mode
            return false;
    }

    return true;
}

/*
  setup manual calibration - based on VL53L1X implementation
  This is called after the first successful reading to configure VHV and phasecal
*/
bool AP_RangeFinder_VL53L3C::setupManualCalibration(void)
{
    printf("VL53L3C: setupManualCalibration() - CALLED\n");

    uint8_t vhv_init_byte;
    uint8_t vhv_timeout;

    // Read VHV init byte and timeout
    if (!read_register(VHV_CONFIG__INIT, vhv_init_byte) ||
        !read_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, vhv_timeout)) {
        printf("VL53L3C: setupManualCalibration() - Failed to read VHV registers\n");
        return false;
    }

    printf("VL53L3C: setupManualCalibration() - vhv_init=0x%02X, vhv_timeout=0x%02X\n",
           vhv_init_byte, vhv_timeout);

    // Override phasecal
    uint8_t phasecal_result_vcsel_start;
    if (!read_register(PHASECAL_RESULT__VCSEL_START, phasecal_result_vcsel_start) ||
        !write_register(CAL_CONFIG__VCSEL_START, phasecal_result_vcsel_start) ||
        !write_register(PHASECAL_CONFIG__OVERRIDE, 0x01)) {
        printf("VL53L3C: setupManualCalibration() - Failed phasecal override sequence\n");
        return false;
    }

    printf("VL53L3C: setupManualCalibration() - SUCCESS, phasecal_vcsel_start=0x%02X\n",
           phasecal_result_vcsel_start);

    return true;
}

// check if data is ready - VL53L3C uses different GPIO status bits
bool AP_RangeFinder_VL53L3C::dataReady(void)
{
    uint8_t gpio_tio_hv_status = 0;

    if (!read_register(GPIO__TIO_HV_STATUS, gpio_tio_hv_status)) {
        return false;
    }

    // VL53L3C: Data ready when bits 0-1 are set (0x02 or 0x03)
    // Different from VL53L1X which checks bit 0 == 0
    bool ready = (gpio_tio_hv_status & 0x03) != 0;

    return ready;
}

// Read distance measurement
bool AP_RangeFinder_VL53L3C::get_reading(uint16_t &reading_mm)
{
    // Increased timeout for long-range measurements
    // Bare driver uses 2000ms, we use 200ms as a balance between responsiveness and accuracy
    // Long distance measurements take more time to complete
    uint16_t tries = 200;
    while (!dataReady()) {
        tries--;
        hal.scheduler->delay(1);
        if (tries == 0) {
            return false;
        }
    }

    uint8_t range_status = 0;

    if (!(read_register(RESULT__RANGE_STATUS, range_status) &&
          read_register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, reading_mm))) {
        return false;
    }

    // DEBUG: Show raw value before correction
    uint16_t raw_reading_mm = reading_mm;

    if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    // VL53L3C: Apply mask to get lower 5 bits only (same as bare driver vl53lx_api.c:741)
    // The upper bits contain additional flags that should be ignored
    uint8_t raw_status = range_status;
    range_status = range_status & 0x1F;

    // VL53L3C Histogram mode: Accept valid device error codes
    // Based on bare driver testing and observation:
    // - DeviceError 9 (RANGECOMPLETE) is ideal but rarely seen in register reads
    // - DeviceError 7 (PHASECONSISTENCY) common, values valid
    // - DeviceError 5 (RANGEPHASECHECK) appears at longer distances, values may be valid
    DeviceError status = (DeviceError)range_status;

    // DEBUG: Print ALL range_status values to understand what sensor is returning
    printf("VL53L3C: raw=0x%02X masked=%u RAW=%u CORRECTED=%u mm\n", raw_status, range_status, raw_reading_mm, reading_mm);

    // Accept DeviceError 9, 7, and 5
    // Testing shows these produce valid distance measurements
    if (status != RANGECOMPLETE && status != PHASECONSISTENCY && status != RANGEPHASECHECK) {
        printf("VL53L3C: Invalid range_status=%u (expected 9, 7, or 5)\n", range_status);
        return false;
    }

    // TEMPORARY: Disable manual calibration to test stability
    // Manual calibration may be causing sensor to hang
    // if (!calibrated) {
    //     if (setupManualCalibration()) {
    //         calibrated = true;
    //         printf("VL53L3C: Manual calibration SUCCESS\n");
    //     } else {
    //         printf("VL53L3C: Manual calibration FAILED\n");
    //     }
    // }

    return true;
}

// Timer callback for periodic measurements
void AP_RangeFinder_VL53L3C::timer(void)
{
    uint16_t range_mm;
    bool read_success = get_reading(range_mm);

    // VL53L3C can measure up to 5m+, extend limit to 6m
    if (read_success && (range_mm <= 6000)) {
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
        printf("VL53L3C: [OK range=%u cnt=%lu]\n", range_mm, (unsigned long)counter);
    } else {
        printf("VL53L3C: [FAIL success=%d range=%u]\n", read_success, range_mm);
    }
}

// update state based on accumulated measurements
void AP_RangeFinder_VL53L3C::update(void)
{
    printf("VL53L3C: update() - CALLED\n");

    WITH_SEMAPHORE(_sem);

    printf("VL53L3C: update() - counter=%lu, sum_mm=%lu\n", (unsigned long)counter, (unsigned long)sum_mm);

    if (counter > 0) {
        float distance_m = (sum_mm / counter) * 0.001f;
        printf("VL53L3C: update() - Calculated distance_m=%.3f (sum_mm=%lu / counter=%lu)\n",
               distance_m, (unsigned long)sum_mm, (unsigned long)counter);

        state.distance_m = distance_m;
        state.last_reading_ms = AP_HAL::millis();

        printf("VL53L3C: update() - Calling update_status(), current status=%d\n",
               (int)state.status);

        update_status();  // Call before resetting counters

        printf("VL53L3C: update() - After update_status(), new status=%d\n",
               (int)state.status);

        sum_mm = 0;
        counter = 0;
    } else {
        uint32_t now = AP_HAL::millis();
        uint32_t time_since_reading = now - state.last_reading_ms;
        printf("VL53L3C: update() - counter=0, time_since_last_reading=%lu ms\n",
               (unsigned long)time_since_reading);

        if (time_since_reading > 200) {
            // no new data for 200ms (same as VL53L1X)
            printf("VL53L3C: update() - Setting status to NoData (timeout > 200ms)\n");
            set_status(RangeFinder::Status::NoData);
        }
    }
}

/*
  I2C register access functions
 */
bool AP_RangeFinder_VL53L3C::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };

    if (!dev->transfer(b, 2, &value, 1)) {
        return false;
    }
    return true;
}

bool AP_RangeFinder_VL53L3C::read_register16(uint16_t reg, uint16_t &value)
{
    uint8_t b[2] { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    uint8_t v[2];

    if (!dev->transfer(b, 2, v, 2)) {
        return false;
    }
    value = (uint16_t(v[0]) << 8) | v[1];
    return true;
}

bool AP_RangeFinder_VL53L3C::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };

    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L3C::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };

    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L3C::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] {
        uint8_t(reg >> 8), uint8_t(reg & 0xFF),
        uint8_t((value >> 24) & 0xFF),
        uint8_t((value >> 16) & 0xFF),
        uint8_t((value >> 8) & 0xFF),
        uint8_t(value & 0xFF)
    };

    return dev->transfer(b, 6, nullptr, 0);
}

/*
  Timing and timeout functions - same as VL53L1X
 */
uint32_t AP_RangeFinder_VL53L3C::calcMacroPeriod(uint8_t vcsel_period) const
{
    // Step 1: Calculate PLL period
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // Step 2: Decode VCSEL period
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // Step 3: Calculate macro period (formula from bare driver)
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    uint32_t step1 = macro_period_us;  // Save for debug
    macro_period_us >>= 6;  // Divide by 64
    uint32_t step2 = macro_period_us;  // Save for debug
    macro_period_us *= vcsel_period_pclks;
    uint32_t step3 = macro_period_us;  // Save for debug
    macro_period_us >>= 6;  // Divide by 64 again

    // Debug output (only print occasionally to avoid spam)
    static uint32_t debug_counter = 0;
    if (debug_counter++ < 3) {
        printf("VL53L3C: calcMacroPeriod(vcsel_reg=%u):\n", vcsel_period);
        printf("  pll_period = 2^30 / %u = %lu\n", fast_osc_frequency, (unsigned long)pll_period_us);
        printf("  vcsel_decoded = (%u + 1) << 1 = %u\n", vcsel_period, vcsel_period_pclks);
        printf("  step1: 2304 * %lu = %lu\n", (unsigned long)pll_period_us, (unsigned long)step1);
        printf("  step2: %lu >> 6 = %lu\n", (unsigned long)step1, (unsigned long)step2);
        printf("  step3: %lu * %u = %lu\n", (unsigned long)step2, vcsel_period_pclks, (unsigned long)step3);
        printf("  final: %lu >> 6 = %lu us\n", (unsigned long)step3, (unsigned long)macro_period_us);
    }

    return macro_period_us;
}

uint32_t AP_RangeFinder_VL53L3C::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t AP_RangeFinder_VL53L3C::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint16_t AP_RangeFinder_VL53L3C::encodeTimeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks <= 0) {
        return 0;
    }

    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
}

uint32_t AP_RangeFinder_VL53L3C::decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

bool AP_RangeFinder_VL53L3C::getMeasurementTimingBudget(uint32_t &budget_us)
{
    uint16_t range_config_timeout;
    uint8_t vcsel_period_a;

    if (!(read_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, range_config_timeout) &&
          read_register(RANGE_CONFIG__VCSEL_PERIOD_A, vcsel_period_a))) {
        return false;
    }

    uint32_t macro_period_us = calcMacroPeriod(vcsel_period_a);
    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(range_config_timeout), macro_period_us);

    budget_us = 2 * range_config_timeout_us + TimingGuard;
    return true;
}

bool AP_RangeFinder_VL53L3C::setMeasurementTimingBudget(uint32_t budget_us)
{
    if (budget_us <= TimingGuard) {
        return false;
    }

    uint32_t range_config_timeout_us = budget_us - TimingGuard;
    if (range_config_timeout_us > 1100000) {
        return false;
    }

    range_config_timeout_us /= 2;

    uint8_t vcsel_period_a, vcsel_period_b;
    if (!(read_register(RANGE_CONFIG__VCSEL_PERIOD_A, vcsel_period_a) &&
          read_register(RANGE_CONFIG__VCSEL_PERIOD_B, vcsel_period_b))) {
        return false;
    }

    // Calculate and write PHASECAL_CONFIG__TIMEOUT_MACROP
    // From bare driver vl53lx_api_core.c line 392: phasecal_config_timeout_us = 1000
    uint32_t phasecal_config_timeout_us = 1000;  // Fixed 1ms for phase calibration
    uint32_t macro_period_us = calcMacroPeriod(vcsel_period_a);
    uint32_t timeout_mclks = timeoutMicrosecondsToMclks(phasecal_config_timeout_us, macro_period_us);
    uint16_t timeout_encoded = encodeTimeout(timeout_mclks);

    printf("VL53L3C: PHASECAL timeout: %luus -> %lu mclks -> 0x%02X\n",
           (unsigned long)phasecal_config_timeout_us, (unsigned long)timeout_mclks, (unsigned int)timeout_encoded);

    if (!(write_register(PHASECAL_CONFIG__TIMEOUT_MACROP, (uint8_t)timeout_encoded))) {
        return false;
    }

    // Calculate and write MM_CONFIG timeout macrop registers
    // From bare driver vl53lx_api_core.c line 393: mm_config_timeout_us = 2000
    uint32_t mm_config_timeout_us = 2000;  // Fixed 2ms for memory management
    macro_period_us = calcMacroPeriod(vcsel_period_a);
    timeout_mclks = timeoutMicrosecondsToMclks(mm_config_timeout_us, macro_period_us);
    timeout_encoded = encodeTimeout(timeout_mclks);

    printf("VL53L3C: MM_CONFIG_A timeout: %luus -> %lu mclks -> 0x%04X (vcsel_a=%u, macro=%luus)\n",
           (unsigned long)mm_config_timeout_us, (unsigned long)timeout_mclks, timeout_encoded,
           vcsel_period_a, (unsigned long)macro_period_us);

    if (!(write_register16(MM_CONFIG__TIMEOUT_MACROP_A, timeout_encoded))) {
        return false;
    }

    macro_period_us = calcMacroPeriod(vcsel_period_b);
    timeout_mclks = timeoutMicrosecondsToMclks(mm_config_timeout_us, macro_period_us);
    timeout_encoded = encodeTimeout(timeout_mclks);

    printf("VL53L3C: MM_CONFIG_B timeout: %luus -> %lu mclks -> 0x%04X (vcsel_b=%u, macro=%luus)\n",
           (unsigned long)mm_config_timeout_us, (unsigned long)timeout_mclks, timeout_encoded,
           vcsel_period_b, (unsigned long)macro_period_us);

    if (!(write_register16(MM_CONFIG__TIMEOUT_MACROP_B, timeout_encoded))) {
        return false;
    }

    // Calculate and write RANGE_CONFIG timeout macrop registers
    macro_period_us = calcMacroPeriod(vcsel_period_a);
    timeout_mclks = timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us);
    timeout_encoded = encodeTimeout(timeout_mclks);

    printf("VL53L3C: RANGE_CONFIG_A timeout: %luus -> %lu mclks -> 0x%04X\n",
           (unsigned long)range_config_timeout_us, (unsigned long)timeout_mclks, timeout_encoded);

    if (!(write_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, timeout_encoded))) {
        return false;
    }

    macro_period_us = calcMacroPeriod(vcsel_period_b);
    timeout_mclks = timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us);
    timeout_encoded = encodeTimeout(timeout_mclks);

    printf("VL53L3C: RANGE_CONFIG_B timeout: %luus -> %lu mclks -> 0x%04X\n",
           (unsigned long)range_config_timeout_us, (unsigned long)timeout_mclks, timeout_encoded);

    if (!(write_register16(RANGE_CONFIG__TIMEOUT_MACROP_B, timeout_encoded))) {
        return false;
    }

    return true;
}

bool AP_RangeFinder_VL53L3C::startContinuous(uint32_t period_ms)
{
    printf("VL53L3C: startContinuous() - CALLED with period_ms=%lu\n", (unsigned long)period_ms);

    // Add timing adjustment (6.4%) as per VL53L1X
    uint32_t adjusted_period_ms = period_ms + (period_ms * 64 / 1000);
    printf("VL53L3C: startContinuous() - adjusted_period_ms=%lu\n", (unsigned long)adjusted_period_ms);

    uint32_t intermeasurement_value = adjusted_period_ms * osc_calibrate_val;
    printf("VL53L3C: startContinuous() - intermeasurement_value=%lu (adjusted_period=%lu * osc_cal=%u)\n",
           (unsigned long)intermeasurement_value, (unsigned long)adjusted_period_ms, osc_calibrate_val);

    // Set inter-measurement period
    if (!write_register32(SYSTEM__INTERMEASUREMENT_PERIOD, intermeasurement_value)) {
        printf("VL53L3C: startContinuous() - FAILED to write SYSTEM__INTERMEASUREMENT_PERIOD\n");
        return false;
    }
    printf("VL53L3C: startContinuous() - Wrote SYSTEM__INTERMEASUREMENT_PERIOD successfully\n");

    if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        printf("VL53L3C: startContinuous() - FAILED to write SYSTEM__INTERRUPT_CLEAR\n");
        return false;
    }
    printf("VL53L3C: startContinuous() - Cleared interrupt successfully\n");

    if (!write_register(SYSTEM__MODE_START, 0x40)) {
        printf("VL53L3C: startContinuous() - FAILED to write SYSTEM__MODE_START\n");
        return false;
    }
    printf("VL53L3C: startContinuous() - Started continuous mode successfully\n");

    return true;
}

#endif  // AP_RANGEFINDER_VL53L3C_ENABLED

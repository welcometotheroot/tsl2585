#ifndef TSL2585_H
#define TSL2585_H

/**
 * @file tsl2585.h
 * @brief Arduino driver for the ams OSRAM TSL2585 tri-channel optical sensor.
 *
 * The TSL2585 is a general-purpose ambient-light sensor with three independent
 * measurement channels:
 *   - Photopic  (Mod0) — human-eye spectral response, peaks ~555nm
 *   - UV        (Mod1) — UV-A band, peaks ~365nm
 *   - IR        (Mod2) — near-infrared, peaks ~940nm
 *
 * Each channel has an independent programmable gain (0.5× to 4096×, 14 steps)
 * and shares a common integration time.
 *
 * Irradiance is returned in mW/cm² using factory responsivity constants from
 * Figure 6 of the TSL2585 datasheet.  A factory OTP calibration byte
 * (UV_CALIB) is applied to the UV channel at `begin()` to compensate for
 * device-to-device variation.
 *
 * ## Typical usage (interrupt-driven)
 * @code
 *   TSL2585::begin(Wire, VIZ_INT_PIN);   // attach ISR, enable ALS interrupt
 *
 *   loop() {
 *     if (TSL2585::isDataReady()) {
 *       TSL2585Data d;
 *       if (TSL2585::read(d)) {
 *         Serial.printf("UV: %.4f mW/cm²\n", d.uvIrradiance);
 *       }
 *     }
 *   }
 * @endcode
 *
 * ## Typical usage (polling)
 * @code
 *   TSL2585::begin(Wire);   // no interrupt pin — caller polls isDataReady()
 * @endcode
 *
 * ## Flicker detection
 * Raw Photopic samples are streamed into the on-chip FIFO.  The caller reads
 * them into a buffer and performs FFT on the host CPU to determine flicker
 * frequency.
 * @code
 *   TSL2585::beginFlickerDetection(128);  // collect 128 samples, loop forever
 *   while (!TSL2585::isFifoReady()) {}
 *   uint8_t buf[256];
 *   uint16_t n;
 *   TSL2585::readFifoSamples(buf, sizeof(buf), n);
 *   // FFT buf[0..n-1]
 * @endcode
 */

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

/**
 * Calibrated measurement result from one ALS cycle.
 *
 * Irradiance values are in mW/cm², consistent with the UVData and VisibleData
 * structs used by the rest of this firmware.
 *
 * Gain fields report the hardware gain actually used for the cycle (read from
 * ALS_STATUS2/3 after each cycle), not the commanded gain codes.  These are
 * the numeric multipliers (e.g. 128), not the 4-bit register codes.
 * Exception: a gain of 0 indicates 0.5× (code 0x00) — callers may treat this
 * as 1 for display purposes.
 *
 * Saturation flags are set when the photodiode output clipped before reaching
 * the ADC.  When a channel is saturated its irradiance value is a lower bound,
 * not an accurate measurement.
 */
struct TSL2585Data {
  float photopicIrradiance;  // mW/cm² — human-eye response
  float uvIrradiance;        // mW/cm² — UV-A, factory-calibrated
  float irIrradiance;        // mW/cm² — near-infrared

  int photopicGain;  // effective gain multiplier (e.g. 128)
  int uvGain;
  int irGain;

  bool photopicSaturated;
  bool uvSaturated;
  bool irSaturated;

  // Raw 16-bit ADC counts as latched on this cycle.  Used by handler-level
  // auto-gain to assess headroom (counts / 65535).  UV is the pre-UV_CALIB
  // value; the calibration correction is folded into uvIrradiance only.
  uint16_t photopicCounts;
  uint16_t uvCounts;
  uint16_t irCounts;
};

/**
 * TSL2585 driver namespace.
 *
 * All state is held in file-scope statics inside tsl2585.cpp — the driver is a
 * singleton intended for a single sensor instance on one I2C bus.
 */
namespace TSL2585 {

/**
 * Initialise the TSL2585 and begin continuous ALS measurement.
 *
 * Waits for INIT_BUSY to clear, performs a soft reset, verifies device ID,
 * reads the factory UV_CALIB OTP byte, configures isolated SMUX, sets 28ms
 * integration, sets all gains to 128×, enables the ALS interrupt on the
 * supplied pin (active-low, open-drain — attach as FALLING), then powers on.
 *
 * @param wire   I2C bus the sensor is on.
 * @param intPin GPIO pin connected to the sensor INT line (active-low).
 * @return true on success; false if the device is not found or misconfigures.
 */
bool begin(TwoWire& wire, uint8_t intPin);

/**
 * Initialise without an interrupt pin.  The caller must poll isDataReady()
 * (which in this mode checks ALS_DATA_VALID in STATUS2 each call rather than
 * relying on an ISR flag).
 *
 * @param wire  I2C bus the sensor is on.
 * @return true on success; false on device or communication error.
 */
bool begin(TwoWire& wire);

// ---------------------------------------------------------------------------
// Data-ready interface
// ---------------------------------------------------------------------------

/**
 * Returns true when a complete ALS cycle result is waiting to be read.
 *
 * In interrupt mode: set by the ISR on the falling edge of INT, cleared by
 * read().  In polling mode: reads STATUS2.ALS_DATA_VALID each call.
 */
bool isDataReady();

/** Called from the ISR — marks data as ready.  Not for normal caller use. */
void setDataReady();

/** Clears the data-ready flag without reading.  Useful after discarding a stale cycle. */
void clearDataReady();

// ---------------------------------------------------------------------------
// Primary read
// ---------------------------------------------------------------------------

/**
 * Read a complete ALS cycle into `data`.
 *
 * Reads ALS_STATUS (latches data registers), verifies data is valid, reads all
 * three 16-bit channels, reads actual gain used from ALS_STATUS2/3, clears the
 * ALS interrupt (restarting the next cycle), applies factory UV calibration,
 * converts counts to mW/cm², and populates `data`.
 *
 * Returns false if the I2C communication fails or if ALS_DATA_VALID is not set
 * (i.e. a cycle has not completed since the last read).
 */
bool read(TSL2585Data& data);

// ---------------------------------------------------------------------------
// Saturation
// ---------------------------------------------------------------------------

/**
 * Returns true if any channel was saturated on the most recent read() call.
 * Use per-channel `TSL2585Data::*Saturated` fields for channel-level detail.
 */
bool checkSaturation();

// ---------------------------------------------------------------------------
// Per-channel gain control
// ---------------------------------------------------------------------------

/**
 * Gain code range:
 *   0x00 = 0.5×,  0x01 = 1×,   0x02 = 2×,   0x03 = 4×,
 *   0x04 = 8×,    0x05 = 16×,  0x06 = 32×,  0x07 = 64×,
 *   0x08 = 128×,  0x09 = 256×, 0x0A = 512×, 0x0B = 1024×,
 *   0x0C = 2048×, 0x0D = 4096×
 *
 * All three gain codes are packed into two register writes by the hardware —
 * calling any setter writes all three commanded codes simultaneously.
 *
 * Returns false if the gain code is out of range or the I2C write fails.
 */
bool setPhotopicGain(uint8_t gainCode);
bool setUVGain(uint8_t gainCode);
bool setIRGain(uint8_t gainCode);

/**
 * Set all three channels to the same gain code in a single write.
 */
bool setAllGains(uint8_t gainCode);

/**
 * Return the currently commanded gain code for each channel.
 * Note: the hardware may have used a different code (hardware AGC, if enabled);
 * the actual code used is reported in TSL2585Data::*Gain after each read().
 */
uint8_t getPhotopicGainCode();
uint8_t getUVGainCode();
uint8_t getIRGainCode();

// ---------------------------------------------------------------------------
// Integration time
// ---------------------------------------------------------------------------

/**
 * Set the number of 250µs samples per ALS cycle (ALS_NR_SAMPLES register).
 *
 * Integration time = (numberOfSamples + 1) × 250µs.
 * Default: 111 → 28.0ms.
 * Maximum: 2047 (11-bit field) → 512ms.
 *
 * @param numberOfSamples  The ALS_NR_SAMPLES value to write (0–2047).
 * @return false on I2C failure.
 */
bool setIntegrationSamples(uint16_t numberOfSamples);

/**
 * Return the current integration time in milliseconds, derived from the
 * stored ALS_NR_SAMPLES value and the fixed 250µs sample step.
 */
uint32_t getIntegrationTimeMs();

// ---------------------------------------------------------------------------
// ALS threshold interrupt
// ---------------------------------------------------------------------------

/**
 * Set the ALS interrupt threshold for a specific channel.
 *
 * @param channel  0 = Photopic (Mod0), 1 = UV (Mod1), 2 = IR (Mod2).
 * @param low      Lower threshold (24-bit).  Interrupt fires when count < low.
 * @param high     Upper threshold (24-bit).  Interrupt fires when count > high.
 * @return false on invalid channel or I2C failure.
 */
bool setAlsThreshold(uint8_t channel, uint32_t low, uint32_t high);

/**
 * Set the ALS persistence filter (APERS field in CFG5).
 *
 * @param persistence  0 = interrupt on every cycle.
 *                     1–15 = interrupt only after N consecutive out-of-range cycles.
 */
bool setAlsPersistence(uint8_t persistence);

// ---------------------------------------------------------------------------
// Flicker detection (raw FIFO)
// ---------------------------------------------------------------------------

/**
 * Begin Photopic flicker detection.  Configures FD_NR_SAMPLES, enables FDEN
 * and FIFO interrupt.  Samples accumulate in the on-chip FIFO; the caller reads
 * them via readFifoSamples() and performs FFT externally.
 *
 * @param sampleCount     Number of samples per collection window.
 * @param infiniteRepeat  If true, the FIFO refills continuously without stopping.
 */
bool beginFlickerDetection(uint16_t sampleCount, bool infiniteRepeat = true);

/** Disable flicker detection (clear FDEN bit). */
void stopFlickerDetection();

/** Returns true when the FIFO contains at least one byte. */
bool isFifoReady();

/** Returns the number of bytes currently in the FIFO (REG_FIFO_STATUS0). */
uint16_t getFifoLevel();

/**
 * Read bytes from the FIFO into caller-supplied buffer.
 *
 * Reads min(getFifoLevel(), maxBytes) bytes.  Returns false and logs a warning
 * if FIFO overflow or underflow flags are set (samples may be corrupt).
 *
 * @param buffer    Destination buffer.
 * @param maxBytes  Size of buffer in bytes.
 * @param bytesRead Set to the number of bytes actually read.
 */
bool readFifoSamples(uint8_t* buffer, uint16_t maxBytes, uint16_t& bytesRead);

/** Clear the FIFO (set CONTROL.FIFO_CLR). */
void clearFifo();

// ---------------------------------------------------------------------------
// Diagnostics and calibration
// ---------------------------------------------------------------------------

/**
 * Read the device ID register.  Expected value is 0x5C (DEVICE_ID).
 * Returns false on I2C failure.
 */
bool getDeviceId(uint8_t& id);

/**
 * Return the raw factory UV calibration OTP byte read at begin().
 * Value 127 = 0% correction (identity).  Applied automatically in read().
 */
uint8_t getUVCalib();

/**
 * Read raw 16-bit ADC counts for all three channels.
 *
 * Triggers the data latch (reads ALS_STATUS) and reads DATA0/1/2.  Useful for
 * calibration workflows where irradiance conversion must be done externally.
 *
 * @param photopic  Output: Photopic (Mod0) raw counts.
 * @param uv        Output: UV (Mod1) raw counts (before UV_CALIB correction).
 * @param ir        Output: IR (Mod2) raw counts.
 * @return false on I2C failure.
 */
bool readRawCounts(uint32_t& photopic, uint32_t& uv, uint32_t& ir);

}  // namespace TSL2585

#endif  // TSL2585_H

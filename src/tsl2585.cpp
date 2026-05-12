#include "tsl2585.h"
#include "tsl2585_registers.h"

// ---------------------------------------------------------------------------
// Static state
// ---------------------------------------------------------------------------

static TwoWire*      s_wire             = nullptr;
static uint8_t       s_intPin           = 0xFF;
static volatile bool s_dataReady        = false;

// Per-channel commanded gain codes (0x00–0x0D). Hardware packs all three into
// two registers; any setter writes all three simultaneously.
static uint8_t  s_photopicGainCode  = 0x08;  // 128×
static uint8_t  s_uvGainCode        = 0x08;  // 128×
static uint8_t  s_irGainCode        = 0x08;  // 128×

// Factory UV_CALIB OTP byte; 127 = 0% correction (identity).
static uint8_t  s_uvCalibByte       = 127;

// ALS_NR_SAMPLES register value (11-bit); integration = (value+1) × 250µs.
static uint16_t s_numberOfSamples   = 111;  // 28ms

// Saturation state from the most recent read() call.
static bool s_photopicSaturated = false;
static bool s_uvSaturated       = false;
static bool s_irSaturated       = false;

// Polling mode flag (no interrupt pin supplied to begin()).
static bool s_pollingMode = false;

static constexpr uint8_t I2C_ADDRESS = 0x39;

// ---------------------------------------------------------------------------
// Irradiance constants (datasheet Figure 6)
// ---------------------------------------------------------------------------

// Responsivity in counts/(µW/cm²) at 128× gain and 10ms integration.
// UV is normalised from the datasheet's 1024× reference: 82.8 / 7.42 ≈ 11.16.
static constexpr float RESPONSIVITY_PHOTOPIC_128X_10MS = 73.7f;
static constexpr float RESPONSIVITY_IR_128X_10MS       = 74.8f;
static constexpr float RESPONSIVITY_UV_128X_10MS       = 11.16f;

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

static void dataReadyISR() {
  s_dataReady = true;
}

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

static bool writeReg(uint8_t reg, uint8_t value) {
  s_wire->beginTransmission(I2C_ADDRESS);
  s_wire->write(reg);
  s_wire->write(value);
  return s_wire->endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint8_t& value) {
  s_wire->beginTransmission(I2C_ADDRESS);
  s_wire->write(reg);
  if (s_wire->endTransmission(false) != 0) return false;
  if (s_wire->requestFrom(I2C_ADDRESS, static_cast<uint8_t>(1)) != 1) return false;
  value = s_wire->read();
  return true;
}

// Read two bytes with auto-increment. Low byte at `reg` triggers the 16-bit
// latch for that channel pair; high byte is at reg+1.
static bool readReg16(uint8_t reg, uint16_t& value) {
  s_wire->beginTransmission(I2C_ADDRESS);
  s_wire->write(reg);
  if (s_wire->endTransmission(false) != 0) return false;
  if (s_wire->requestFrom(I2C_ADDRESS, static_cast<uint8_t>(2)) != 2) return false;
  uint8_t lo = s_wire->read();
  uint8_t hi = s_wire->read();
  value = static_cast<uint16_t>(static_cast<uint16_t>(hi) << 8) | lo;
  return true;
}

// Write all three channel gain codes in a single two-register burst.
// 0xD4 [7:4] = UV (Mod1), [3:0] = Photopic (Mod0)
// 0xD5 [3:0] = IR (Mod2)
static bool writeGainRegisters(uint8_t photopicCode, uint8_t uvCode, uint8_t irCode) {
  uint8_t gainL = static_cast<uint8_t>((uvCode << 4) | (photopicCode & 0x0F));
  return writeReg(REG_STEP0_GAIN_L, gainL) &&
         writeReg(REG_STEP0_GAIN_H, irCode & 0x0F);
}

// ---------------------------------------------------------------------------
// Irradiance conversion helpers
// ---------------------------------------------------------------------------

// Returns the numeric gain multiplier for a gain code. Code 0x00 = 0.5×,
// returned as 0 — callers that display this may treat 0 as 1.
static int gainCodeToValue(uint8_t code) {
  if (code == 0) return 0;  // 0.5×
  return (1 << (code - 1));
}

static float gainCodeToFloat(uint8_t code) {
  if (code == 0) return 0.5f;
  return static_cast<float>(1 << (code - 1));
}

// Convert raw ADC counts to µW/cm² at any gain and integration time.
static float countsToIrradiance(float counts, uint8_t gainCode,
                                 float integrationMs, float responsivityAt128x10ms) {
  float gain = gainCodeToFloat(gainCode);
  return counts / (responsivityAt128x10ms * (gain / 128.0f) * (integrationMs / 10.0f));
}

// ---------------------------------------------------------------------------
// begin() — shared init body
// ---------------------------------------------------------------------------

static bool initSensor() {
  // 1. Wait for INIT_BUSY to clear (up to 5ms).
  uint32_t deadline = millis() + 5;
  uint8_t status4;
  do {
    if (!readReg(REG_STATUS4, status4)) {
      S.println("TSL2585: I2C error waiting for INIT_BUSY");
      return false;
    }
    if (!(status4 & STATUS4_INIT_BUSY)) break;
  } while (millis() < deadline);

  if (status4 & STATUS4_INIT_BUSY) {
    S.println("TSL2585: INIT_BUSY timeout");
    return false;
  }

  // 2. Soft reset with oscillator on (required by datasheet).
  if (!writeReg(REG_ENABLE, ENABLE_PON)) {
    S.println("TSL2585: Failed to power on for reset");
    return false;
  }
  delay(1);
  if (!writeReg(REG_CONTROL, CONTROL_SOFT_RESET)) {
    S.println("TSL2585: Soft reset failed");
    return false;
  }
  delay(1);

  // 3. Verify device ID.
  uint8_t id;
  if (!readReg(REG_ID, id)) {
    S.println("TSL2585: Failed to read device ID");
    return false;
  }
  if (id != DEVICE_ID) {
    S.printf("TSL2585: Unexpected device ID 0x%02X (expected 0x%02X)\n", id, DEVICE_ID);
    return false;
  }

  // 4. Read factory UV calibration OTP byte.
  if (!readReg(REG_UV_CALIB, s_uvCalibByte)) {
    S.println("TSL2585: Failed to read UV_CALIB");
    return false;
  }

  // 5. Disable ALS_SCALE for direct 16-bit readout (clear bits [3:0] of MEAS_MODE0).
  uint8_t measMode0;
  if (!readReg(REG_MEAS_MODE0, measMode0)) {
    S.println("TSL2585: Failed to read MEAS_MODE0");
    return false;
  }
  if (!writeReg(REG_MEAS_MODE0, measMode0 & 0xF0)) {
    S.println("TSL2585: Failed to write MEAS_MODE0");
    return false;
  }

  // 6. Configure SMUX for isolated channels (step 0 only):
  //    Mod0 = Photopic (PD1+PD5), Mod1 = UV (PD3+PD4), Mod2 = IR (PD0+PD2).
  if (!writeReg(REG_STEP0_SMUX_L, SMUX_L_ISOLATED) ||
      !writeReg(REG_STEP0_SMUX_H, SMUX_H_ISOLATED)) {
    S.println("TSL2585: Failed to configure SMUX");
    return false;
  }

  // 7. Set integration time to 28ms (ALS_NR_SAMPLES = 111).
  s_numberOfSamples = 111;
  if (!writeReg(REG_ALS_NR_SAMPLES0, 0x6F) ||
      !writeReg(REG_ALS_NR_SAMPLES1, 0x00)) {
    S.println("TSL2585: Failed to set integration time");
    return false;
  }

  // 8. Set initial gain 128× (code 0x08) on all three channels.
  s_photopicGainCode = s_uvGainCode = s_irGainCode = 0x08;
  if (!writeGainRegisters(s_photopicGainCode, s_uvGainCode, s_irGainCode)) {
    S.println("TSL2585: Failed to set gain");
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Public API — Initialisation
// ---------------------------------------------------------------------------

namespace TSL2585 {

bool begin(TwoWire& wire, uint8_t intPin) {
  s_wire = &wire;
  s_intPin = intPin;
  s_pollingMode = false;

  if (!initSensor()) return false;

  // 9. Configure ALS interrupt (active-low, open-drain — FALLING edge).
  attachInterrupt(digitalPinToInterrupt(intPin), dataReadyISR, FALLING);
  if (!writeReg(REG_INTENAB, INTENAB_AIEN)) {
    S.println("TSL2585: Failed to enable ALS interrupt");
    return false;
  }
  // APERS = 0 (interrupt every cycle), threshold channel = Mod0.
  if (!writeReg(REG_CFG5, 0x00)) {
    S.println("TSL2585: Failed to configure CFG5");
    return false;
  }

  // 10. Power on and enable ALS.
  if (!writeReg(REG_ENABLE, ENABLE_PON)) {
    S.println("TSL2585: Failed to power on");
    return false;
  }
  delayMicroseconds(500);
  if (!writeReg(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) {
    S.println("TSL2585: Failed to enable ALS");
    return false;
  }

  uint8_t id;
  readReg(REG_ID, id);
  S.printf("TSL2585 initialised (interrupt mode). ID=0x%02X UV_CALIB=%d\n", id, s_uvCalibByte);
  return true;
}

bool begin(TwoWire& wire) {
  s_wire = &wire;
  s_intPin = 0xFF;
  s_pollingMode = true;

  if (!initSensor()) return false;

  // No interrupt pin — AEN interrupt not enabled; caller polls isDataReady().
  if (!writeReg(REG_ENABLE, ENABLE_PON)) {
    S.println("TSL2585: Failed to power on");
    return false;
  }
  delayMicroseconds(500);
  if (!writeReg(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) {
    S.println("TSL2585: Failed to enable ALS");
    return false;
  }

  uint8_t id;
  readReg(REG_ID, id);
  S.printf("TSL2585 initialised (polling mode). ID=0x%02X UV_CALIB=%d\n", id, s_uvCalibByte);
  return true;
}

// ---------------------------------------------------------------------------
// Data-ready interface
// ---------------------------------------------------------------------------

bool isDataReady() {
  if (s_pollingMode) {
    uint8_t status2;
    if (!readReg(REG_STATUS2, status2)) return false;
    return (status2 & STATUS2_ALS_DATA_VALID) != 0;
  }
  return s_dataReady;
}

void setDataReady() {
  s_dataReady = true;
}

void clearDataReady() {
  s_dataReady = false;
}

// ---------------------------------------------------------------------------
// Primary read
// ---------------------------------------------------------------------------

bool read(TSL2585Data& data) {
  // ALS_DATA_VALID is clear-on-read of STATUS2, so isDataReady() (which polls
  // STATUS2) is the sole gate — don't re-check it here.  Read ALS_STATUS to
  // latch the data registers and capture saturation flags.
  uint8_t alsStatus;
  if (!readReg(REG_ALS_STATUS, alsStatus)) return false;

  // Read three 16-bit channels: DATA0 = Photopic, DATA1 = UV, DATA2 = IR.
  uint16_t rawPhotopic, rawUV, rawIR;
  if (!readReg16(REG_ALS_DATA0_LOW, rawPhotopic)) return false;
  if (!readReg16(REG_ALS_DATA1_LOW, rawUV))       return false;
  if (!readReg16(REG_ALS_DATA2_LOW, rawIR))       return false;

  // Per-channel analog saturation flags from ALS_STATUS.
  bool satPhotopic = (alsStatus & ALS_STATUS_MOD0_ANALOG_SAT) != 0;
  bool satUV       = (alsStatus & ALS_STATUS_MOD1_ANALOG_SAT) != 0;
  bool satIR       = (alsStatus & ALS_STATUS_MOD2_ANALOG_SAT) != 0;

  // Read gain codes actually used by hardware (may differ from commanded codes
  // if hardware AGC is active or if a gain change was mid-cycle).
  uint8_t gainStatus2 = 0, gainStatus3 = 0;
  readReg(REG_ALS_STATUS2, gainStatus2);
  readReg(REG_ALS_STATUS3, gainStatus3);
  uint8_t photopicGainCode = gainStatus2 & 0x0F;
  uint8_t uvGainCode       = (gainStatus2 >> 4) & 0x0F;
  uint8_t irGainCode       = gainStatus3 & 0x0F;

  // Clear the ALS interrupt — this restarts the next measurement cycle.
  writeReg(REG_STATUS, STATUS_AINT);

  // Apply factory UV calibration to UV counts only.
  // Formula: UV_cal = UV_raw / (1 - (UV_CALIB - 127) / 100)
  float calibFactor = 1.0f - (static_cast<float>(s_uvCalibByte) - 127.0f) / 100.0f;
  float calibratedUV = (calibFactor > 0.0f)
                           ? static_cast<float>(rawUV) / calibFactor
                           : static_cast<float>(rawUV);

  // Convert counts → µW/cm² → mW/cm².
  float integrationMs = static_cast<float>(getIntegrationTimeMs());
  data.photopicIrradiance = countsToIrradiance(
      static_cast<float>(rawPhotopic), photopicGainCode,
      integrationMs, RESPONSIVITY_PHOTOPIC_128X_10MS) / 1000.0f;
  data.uvIrradiance = countsToIrradiance(
      calibratedUV, uvGainCode,
      integrationMs, RESPONSIVITY_UV_128X_10MS) / 1000.0f;
  data.irIrradiance = countsToIrradiance(
      static_cast<float>(rawIR), irGainCode,
      integrationMs, RESPONSIVITY_IR_128X_10MS) / 1000.0f;

  data.photopicGain      = gainCodeToValue(photopicGainCode);
  data.uvGain            = gainCodeToValue(uvGainCode);
  data.irGain            = gainCodeToValue(irGainCode);
  data.photopicSaturated = satPhotopic;
  data.uvSaturated       = satUV;
  data.irSaturated       = satIR;
  data.photopicCounts    = rawPhotopic;
  data.uvCounts          = rawUV;
  data.irCounts          = rawIR;

  // Cache saturation state for checkSaturation().
  s_photopicSaturated = satPhotopic;
  s_uvSaturated       = satUV;
  s_irSaturated       = satIR;

  s_dataReady = false;
  return true;
}

// ---------------------------------------------------------------------------
// Saturation
// ---------------------------------------------------------------------------

bool checkSaturation() {
  return s_photopicSaturated || s_uvSaturated || s_irSaturated;
}

// ---------------------------------------------------------------------------
// Per-channel gain
// ---------------------------------------------------------------------------

bool setPhotopicGain(uint8_t gainCode) {
  if (gainCode > MAX_GAIN_CODE) return false;
  s_photopicGainCode = gainCode;
  S.printf("TSL2585: Photopic gain set to code 0x%02X (%d×)\n",
           gainCode, gainCodeToValue(gainCode));
  return writeGainRegisters(s_photopicGainCode, s_uvGainCode, s_irGainCode);
}

bool setUVGain(uint8_t gainCode) {
  if (gainCode > MAX_GAIN_CODE) return false;
  s_uvGainCode = gainCode;
  S.printf("TSL2585: UV gain set to code 0x%02X (%d×)\n",
           gainCode, gainCodeToValue(gainCode));
  return writeGainRegisters(s_photopicGainCode, s_uvGainCode, s_irGainCode);
}

bool setIRGain(uint8_t gainCode) {
  if (gainCode > MAX_GAIN_CODE) return false;
  s_irGainCode = gainCode;
  S.printf("TSL2585: IR gain set to code 0x%02X (%d×)\n",
           gainCode, gainCodeToValue(gainCode));
  return writeGainRegisters(s_photopicGainCode, s_uvGainCode, s_irGainCode);
}

bool setAllGains(uint8_t gainCode) {
  if (gainCode > MAX_GAIN_CODE) return false;
  s_photopicGainCode = s_uvGainCode = s_irGainCode = gainCode;
  S.printf("TSL2585: All gains set to code 0x%02X (%d×)\n",
           gainCode, gainCodeToValue(gainCode));
  return writeGainRegisters(gainCode, gainCode, gainCode);
}

uint8_t getPhotopicGainCode() { return s_photopicGainCode; }
uint8_t getUVGainCode()       { return s_uvGainCode; }
uint8_t getIRGainCode()       { return s_irGainCode; }

// ---------------------------------------------------------------------------
// Integration time
// ---------------------------------------------------------------------------

bool setIntegrationSamples(uint16_t numberOfSamples) {
  uint8_t lo = static_cast<uint8_t>(numberOfSamples & 0xFF);
  uint8_t hi = static_cast<uint8_t>((numberOfSamples >> 8) & 0x07);
  if (!writeReg(REG_ALS_NR_SAMPLES0, lo) ||
      !writeReg(REG_ALS_NR_SAMPLES1, hi)) {
    S.println("TSL2585: Failed to set ALS_NR_SAMPLES");
    return false;
  }
  s_numberOfSamples = numberOfSamples;
  S.printf("TSL2585: Integration time set to %lums (ALS_NR_SAMPLES=%u)\n",
           getIntegrationTimeMs(), numberOfSamples);
  return true;
}

uint32_t getIntegrationTimeMs() {
  // integration_ms = (ALS_NR_SAMPLES + 1) × (SAMPLE_TIME + 1) × 1.388889µs
  // With default SAMPLE_TIME = 179: step = (179+1) × 1.388889µs = 250µs = 0.25ms
  return static_cast<uint32_t>((static_cast<uint32_t>(s_numberOfSamples) + 1) * 250) / 1000;
}

// ---------------------------------------------------------------------------
// ALS threshold interrupt
// ---------------------------------------------------------------------------

bool setAlsThreshold(uint8_t channel, uint32_t low, uint32_t high) {
  if (channel > 2) return false;

  // Write 24-bit low threshold (3 bytes starting at REG_AILT0).
  if (!writeReg(REG_AILT0,     static_cast<uint8_t>(low & 0xFF)) ||
      !writeReg(REG_AILT0 + 1, static_cast<uint8_t>((low >> 8) & 0xFF)) ||
      !writeReg(REG_AILT0 + 2, static_cast<uint8_t>((low >> 16) & 0xFF))) {
    S.println("TSL2585: Failed to write ALS low threshold");
    return false;
  }

  // Write 24-bit high threshold (3 bytes starting at REG_AIHT0).
  if (!writeReg(REG_AIHT0,     static_cast<uint8_t>(high & 0xFF)) ||
      !writeReg(REG_AIHT0 + 1, static_cast<uint8_t>((high >> 8) & 0xFF)) ||
      !writeReg(REG_AIHT0 + 2, static_cast<uint8_t>((high >> 16) & 0xFF))) {
    S.println("TSL2585: Failed to write ALS high threshold");
    return false;
  }

  // Set threshold channel in CFG5 bits [6:4]; preserve APERS in bits [3:0].
  uint8_t cfg5;
  if (!readReg(REG_CFG5, cfg5)) return false;
  cfg5 = (cfg5 & 0x8F) | static_cast<uint8_t>((channel & 0x07) << 4);
  if (!writeReg(REG_CFG5, cfg5)) {
    S.println("TSL2585: Failed to set threshold channel in CFG5");
    return false;
  }

  S.printf("TSL2585: ALS threshold ch%d low=%lu high=%lu\n", channel, low, high);
  return true;
}

bool setAlsPersistence(uint8_t persistence) {
  uint8_t cfg5;
  if (!readReg(REG_CFG5, cfg5)) return false;
  cfg5 = (cfg5 & 0xF0) | (persistence & 0x0F);
  if (!writeReg(REG_CFG5, cfg5)) {
    S.println("TSL2585: Failed to set APERS");
    return false;
  }
  S.printf("TSL2585: APERS set to %d\n", persistence);
  return true;
}

// ---------------------------------------------------------------------------
// Flicker detection
// ---------------------------------------------------------------------------

bool beginFlickerDetection(uint16_t sampleCount, bool infiniteRepeat) {
  if (!writeReg(REG_FD_NR_SAMPLES0, static_cast<uint8_t>(sampleCount & 0xFF))) {
    return false;
  }
  uint8_t hi = static_cast<uint8_t>((sampleCount >> 8) & 0x07);
  if (infiniteRepeat) hi |= static_cast<uint8_t>(1 << 7);
  if (!writeReg(REG_FD_NR_SAMPLES1, hi)) return false;

  clearFifo();

  // Enable FIFO interrupt alongside any existing interrupt enables.
  uint8_t intenab;
  if (!readReg(REG_INTENAB, intenab)) return false;
  if (!writeReg(REG_INTENAB, intenab | INTENAB_FIEN)) return false;

  // Enable flicker detection (FDEN bit).
  uint8_t enable;
  if (!readReg(REG_ENABLE, enable)) return false;
  if (!writeReg(REG_ENABLE, enable | ENABLE_FDEN)) return false;

  S.printf("TSL2585: Flicker detection started (%d samples, %s)\n",
           sampleCount, infiniteRepeat ? "infinite" : "one-shot");
  return true;
}

void stopFlickerDetection() {
  uint8_t enable;
  readReg(REG_ENABLE, enable);
  writeReg(REG_ENABLE, enable & static_cast<uint8_t>(~ENABLE_FDEN));
  S.println("TSL2585: Flicker detection stopped");
}

bool isFifoReady() {
  return getFifoLevel() > 0;
}

uint16_t getFifoLevel() {
  uint8_t level = 0;
  readReg(REG_FIFO_STATUS0, level);
  return static_cast<uint16_t>(level);
}

bool readFifoSamples(uint8_t* buffer, uint16_t maxBytes, uint16_t& bytesRead) {
  bytesRead = min(getFifoLevel(), maxBytes);
  for (uint16_t i = 0; i < bytesRead; i++) {
    if (!readReg(REG_FIFO_DATA, buffer[i])) return false;
  }
  uint8_t fifoStatus1;
  readReg(REG_FIFO_STATUS1, fifoStatus1);
  if (fifoStatus1 & (FIFO_STATUS1_OVERFLOW | FIFO_STATUS1_UNDERFLOW)) {
    S.println("TSL2585: FIFO error — samples may be corrupt");
    return false;
  }
  return true;
}

void clearFifo() {
  writeReg(REG_CONTROL, CONTROL_FIFO_CLR);
}

// ---------------------------------------------------------------------------
// Diagnostics and calibration
// ---------------------------------------------------------------------------

bool getDeviceId(uint8_t& id) {
  return readReg(REG_ID, id);
}

uint8_t getUVCalib() {
  return s_uvCalibByte;
}

bool readRawCounts(uint32_t& photopic, uint32_t& uv, uint32_t& ir) {
  // Read ALS_STATUS to latch all data registers.
  uint8_t alsStatus;
  if (!readReg(REG_ALS_STATUS, alsStatus)) return false;

  uint16_t p, u, r;
  if (!readReg16(REG_ALS_DATA0_LOW, p)) return false;
  if (!readReg16(REG_ALS_DATA1_LOW, u)) return false;
  if (!readReg16(REG_ALS_DATA2_LOW, r)) return false;

  photopic = static_cast<uint32_t>(p);
  uv       = static_cast<uint32_t>(u);
  ir       = static_cast<uint32_t>(r);
  return true;
}

}  // namespace TSL2585

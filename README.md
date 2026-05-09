# TSL2585 Arduino Driver

A standalone Arduino driver for the ams OSRAM TSL2585 tri-channel optical sensor, providing calibrated irradiance measurements across Photopic, UV, and IR channels with per-channel auto-gain, factory UV calibration, flash metering support, and FIFO-based flicker detection.

## Device Overview

The TSL2585 measures light across three independent channels simultaneously:

| Channel  | Modulator | Spectral response     | ALS register |
|----------|-----------|-----------------------|--------------|
| Photopic | Mod0      | Human-eye (~555nm)    | ALS_DATA0    |
| UV       | Mod1      | UV-A (~365nm)         | ALS_DATA1    |
| IR       | Mod2      | Near-infrared (~940nm)| ALS_DATA2    |

Each channel has an independent programmable gain (0.5× to 4096×, 14 steps) and per-channel software auto-gain that keeps all three ADC outputs in the 10–90% full-scale window independently. This is important for spectrally selective sources: a 365nm UV lamp drives UV gain low while IR and Photopic climb to maximum sensitivity without interference.

Irradiance is returned in **mW/cm²** using factory responsivity constants from the TSL2585 datasheet (Figure 6). A factory OTP calibration byte (`UV_CALIB`) is applied to the UV channel at `begin()` to compensate for device-to-device variation.

## Installation

### PlatformIO (recommended)

Clone or add as a submodule into your project's `lib/` directory:

```bash
git submodule add git@github.com:welcometotheroot/tsl2585.git lib/tsl2585
```

PlatformIO resolves `src/` automatically via `library.json` — no `lib_deps` entry needed for local libraries.

### Arduino IDE

Copy the `lib/tsl2585/` folder into your Arduino `libraries/` directory and restart the IDE.

## Usage

### Interrupt-driven (recommended)

```cpp
#include <Wire.h>
#include <tsl2585.h>

static constexpr uint8_t INT_PIN = 2;  // active-low, connect to TSL2585 INT

void setup() {
  Wire.begin();
  if (!TSL2585::begin(Wire, INT_PIN)) {
    Serial.println("TSL2585 not found — check wiring.");
    while (true) {}
  }
}

void loop() {
  if (!TSL2585::isDataReady()) return;

  TSL2585Data d;
  if (TSL2585::read(d)) {
    Serial.print("UV: ");
    Serial.print(d.uvIrradiance, 4);
    Serial.println(" mW/cm²");
  }
}
```

### Polling mode

```cpp
TSL2585::begin(Wire);  // no interrupt pin — isDataReady() polls STATUS2

void loop() {
  if (TSL2585::isDataReady()) {
    TSL2585Data d;
    TSL2585::read(d);
  }
}
```

### Flash metering

Disable auto-gain and switch to fast integration before triggering a flash. Restore afterwards:

```cpp
TSL2585::setAutoGainEnabled(false);
TSL2585::setFastIntegration();      // ~8ms integration

// ... trigger flash, wait for isDataReady() ...

TSL2585Data d;
TSL2585::read(d);
TSL2585::resumeNormalIntegration(); // restores 28ms, re-enables auto-gain
```

### Flicker detection

Raw Photopic samples stream into the on-chip FIFO. Read them into a buffer and perform FFT externally to determine flicker frequency:

```cpp
TSL2585::beginFlickerDetection(128);  // 128 samples, loops forever

while (!TSL2585::isFifoReady()) {}

uint8_t buf[256];
uint16_t n;
TSL2585::readFifoSamples(buf, sizeof(buf), n);
// FFT buf[0..n-1]
```

## API Reference

### Initialisation

| Function | Description |
|----------|-------------|
| `begin(wire, intPin)` | Initialise with interrupt pin (active-low, FALLING edge). |
| `begin(wire)` | Initialise in polling mode — no interrupt pin required. |

### Data ready

| Function | Description |
|----------|-------------|
| `isDataReady()` | Returns true when a complete ALS cycle is ready to read. |
| `setDataReady()` | Called from ISR — marks data as ready. |
| `clearDataReady()` | Clears data-ready flag without reading. |

### Reading

| Function | Description |
|----------|-------------|
| `read(data)` | Read a complete cycle into `TSL2585Data`. Returns false on I2C error or incomplete cycle. |
| `checkSaturation()` | Returns true if any channel was saturated on the last `read()`. |
| `readRawCounts(photopic, uv, ir)` | Read raw 16-bit counts (before irradiance conversion and UV calibration). |

### Gain control

| Function | Description |
|----------|-------------|
| `setAutoGainEnabled(bool)` | Enable/disable per-channel software auto-gain (default: enabled). |
| `setPhotopicGain(code)` | Set Photopic gain code (0x00–0x0D). |
| `setUVGain(code)` | Set UV gain code. |
| `setIRGain(code)` | Set IR gain code. |
| `setAllGains(code)` | Set all three channels to the same gain code. |
| `getPhotopicGainCode()` | Return the commanded Photopic gain code. |
| `getUVGainCode()` | Return the commanded UV gain code. |
| `getIRGainCode()` | Return the commanded IR gain code. |

Gain codes:

| Code | Gain  | Code | Gain  |
|------|-------|------|-------|
| 0x00 | 0.5×  | 0x07 | 64×   |
| 0x01 | 1×    | 0x08 | 128×  |
| 0x02 | 2×    | 0x09 | 256×  |
| 0x03 | 4×    | 0x0A | 512×  |
| 0x04 | 8×    | 0x0B | 1024× |
| 0x05 | 16×   | 0x0C | 2048× |
| 0x06 | 32×   | 0x0D | 4096× |

### Integration time

| Function | Description |
|----------|-------------|
| `setIntegrationSamples(n)` | Set ALS_NR_SAMPLES (0–2047). Integration time = (n+1) × 250µs. |
| `getIntegrationTimeMs()` | Return current integration time in milliseconds. |
| `setFastIntegration()` | Switch to ~8ms integration and disable auto-gain (for flash metering). |
| `resumeNormalIntegration()` | Restore saved integration time and re-enable auto-gain. |

### ALS threshold interrupt

| Function | Description |
|----------|-------------|
| `setAlsThreshold(channel, low, high)` | Set 24-bit low/high threshold for a channel (0=Photopic, 1=UV, 2=IR). |
| `setAlsPersistence(n)` | APERS: 0 = interrupt every cycle; 1–15 = N consecutive out-of-range cycles. |

### Flicker detection

| Function | Description |
|----------|-------------|
| `beginFlickerDetection(sampleCount, infiniteRepeat)` | Start Photopic flicker sampling into FIFO. |
| `stopFlickerDetection()` | Disable flicker detection. |
| `isFifoReady()` | Returns true when FIFO contains at least one byte. |
| `getFifoLevel()` | Returns number of bytes currently in FIFO. |
| `readFifoSamples(buffer, maxBytes, bytesRead)` | Drain FIFO into caller-supplied buffer. |
| `clearFifo()` | Clear the FIFO. |

### Diagnostics

| Function | Description |
|----------|-------------|
| `getDeviceId(id)` | Read device ID register — expected value 0x5C. |
| `getUVCalib()` | Return raw factory UV_CALIB OTP byte (127 = 0% correction). |

## `TSL2585Data` struct

```cpp
struct TSL2585Data {
  float photopicIrradiance;  // mW/cm²
  float uvIrradiance;        // mW/cm² — factory-calibrated
  float irIrradiance;        // mW/cm²

  int photopicGain;          // effective gain multiplier (e.g. 128); 0 = 0.5×
  int uvGain;
  int irGain;

  bool photopicSaturated;    // true if photodiode clipped before ADC
  bool uvSaturated;
  bool irSaturated;
};
```

Gain values report the hardware gain **actually used** for the cycle (read from `ALS_STATUS2/3`), not the commanded code. When a channel is saturated its irradiance value is a lower bound — auto-gain will decrease sensitivity on the next cycle.

## Hardware notes

- **I2C address**: 0x39 (fixed)
- **INT pin**: active-low, open-drain — requires external pull-up. Attach as `FALLING` edge (opposite polarity to the AS7331).
- **Supply voltage**: 1.8V–3.6V
- **Default integration time**: 28ms (`ALS_NR_SAMPLES = 111`, 112 × 250µs steps)
- **Default gain**: 128× on all three channels

## Calibration notes

UV irradiance accuracy depends on two sources of uncertainty:

1. **Device-level**: `UV_CALIB` OTP byte corrects for photodiode process variation. Applied automatically in `read()`.
2. **Responsivity normalisation**: The datasheet specifies UV responsivity at 1024× gain. This driver normalises to 128× using the typical gain ratio of 7.42 (range 6.16–8.68), introducing ±8% uncertainty. This is corrected by solar calibration against a reference meter.

## License

MIT

#ifndef TSL2585_REGISTERS_H
#define TSL2585_REGISTERS_H

#include <stdint.h>

// ams OSRAM TSL2585 register map and named bit constants.
// Include only from tsl2585.cpp — not part of the public API.

// --- Control ---
static constexpr uint8_t REG_UV_CALIB    = 0x08;
static constexpr uint8_t REG_MOD_CH_CTRL = 0x40;
static constexpr uint8_t REG_ENABLE      = 0x80;
static constexpr uint8_t REG_MEAS_MODE0  = 0x81;
static constexpr uint8_t REG_MEAS_MODE1  = 0x82;
static constexpr uint8_t REG_CONTROL     = 0xB1;
static constexpr uint8_t REG_INTENAB     = 0xBA;
static constexpr uint8_t REG_SIEN        = 0xBB;

static constexpr uint8_t ENABLE_PON           = (1 << 0);
static constexpr uint8_t ENABLE_AEN           = (1 << 1);
static constexpr uint8_t ENABLE_FDEN          = (1 << 6);
static constexpr uint8_t CONTROL_SOFT_RESET   = (1 << 3);
static constexpr uint8_t CONTROL_FIFO_CLR     = (1 << 1);
static constexpr uint8_t CONTROL_CLEAR_SAI    = (1 << 0);
static constexpr uint8_t INTENAB_SIEN         = (1 << 0);
static constexpr uint8_t INTENAB_FIEN         = (1 << 2);
static constexpr uint8_t INTENAB_AIEN         = (1 << 3);
static constexpr uint8_t INTENAB_MIEN         = (1 << 7);

// --- Timing ---
static constexpr uint8_t REG_SAMPLE_TIME0     = 0x83;
static constexpr uint8_t REG_SAMPLE_TIME1     = 0x84;
static constexpr uint8_t REG_ALS_NR_SAMPLES0  = 0x85;
static constexpr uint8_t REG_ALS_NR_SAMPLES1  = 0x86;
static constexpr uint8_t REG_FD_NR_SAMPLES0   = 0x87;
static constexpr uint8_t REG_FD_NR_SAMPLES1   = 0x88;
static constexpr uint8_t REG_WTIME            = 0x89;

// Default SAMPLE_TIME value giving 250µs step size (179 = 0xB3).
static constexpr uint8_t SAMPLE_TIME_DEFAULT  = 179;

// --- Thresholds and Config ---
static constexpr uint8_t REG_AILT0  = 0x8A;  // low threshold bytes 0-2: 0x8A..0x8C
static constexpr uint8_t REG_AIHT0  = 0x8D;  // high threshold bytes 0-2: 0x8D..0x8F
static constexpr uint8_t REG_CFG0   = 0xA1;
static constexpr uint8_t REG_CFG5   = 0xA6;  // ALS_THRESHOLD_CHANNEL [6:4], APERS [3:0]

// --- Identity and Status ---
static constexpr uint8_t REG_AUX_ID    = 0x90;
static constexpr uint8_t REG_REV_ID    = 0x91;
static constexpr uint8_t REG_ID        = 0x92;
static constexpr uint8_t DEVICE_ID     = 0x5C;

static constexpr uint8_t REG_STATUS     = 0x93;
static constexpr uint8_t REG_ALS_STATUS = 0x94;
static constexpr uint8_t REG_STATUS2    = 0x9D;  // ALS_DATA_VALID, digital saturation
static constexpr uint8_t REG_STATUS4    = 0x9F;  // INIT_BUSY, SAI_ACTIVE

static constexpr uint8_t STATUS_AINT                = (1 << 3);
static constexpr uint8_t STATUS_FINT                = (1 << 2);
static constexpr uint8_t STATUS_MINT                = (1 << 7);
static constexpr uint8_t ALS_STATUS_MOD0_ANALOG_SAT = (1 << 5);  // Photopic
static constexpr uint8_t ALS_STATUS_MOD1_ANALOG_SAT = (1 << 4);  // UV
static constexpr uint8_t ALS_STATUS_MOD2_ANALOG_SAT = (1 << 3);  // IR
static constexpr uint8_t STATUS2_ALS_DATA_VALID     = (1 << 6);
static constexpr uint8_t STATUS2_ALS_DIGITAL_SAT    = (1 << 4);
static constexpr uint8_t STATUS4_INIT_BUSY          = (1 << 0);

// --- ALS Data Registers ---
static constexpr uint8_t REG_ALS_DATA0_LOW  = 0x95;  // Photopic (Mod0)
static constexpr uint8_t REG_ALS_DATA1_LOW  = 0x97;  // UV (Mod1)
static constexpr uint8_t REG_ALS_DATA2_LOW  = 0x99;  // IR (Mod2)
static constexpr uint8_t REG_ALS_STATUS2    = 0x9B;  // bits [7:4]=Mod1 gain, [3:0]=Mod0 gain
static constexpr uint8_t REG_ALS_STATUS3    = 0x9C;  // bits [3:0]=Mod2 gain

// --- Sequencer Gain (step 0 only) ---
// 0xD4: [7:4] = Mod1 (UV) gain code, [3:0] = Mod0 (Photopic) gain code
// 0xD5: [3:0] = Mod2 (IR) gain code
static constexpr uint8_t REG_STEP0_GAIN_L  = 0xD4;
static constexpr uint8_t REG_STEP0_GAIN_H  = 0xD5;
static constexpr uint8_t MAX_GAIN_CODE     = 0x0D;  // 4096×

// --- SMUX (step 0 only) ---
static constexpr uint8_t REG_STEP0_SMUX_L  = 0xDC;
static constexpr uint8_t REG_STEP0_SMUX_H  = 0xDD;
// Isolated channel mapping: Mod0=Photopic (PD1+PD5), Mod1=UV (PD3+PD4), Mod2=IR (PD0+PD2)
static constexpr uint8_t SMUX_L_ISOLATED   = 0xB7;
static constexpr uint8_t SMUX_H_ISOLATED   = 0x06;

// --- FIFO ---
static constexpr uint8_t REG_FIFO_THR      = 0xFC;
static constexpr uint8_t REG_FIFO_STATUS0  = 0xFD;  // FIFO level (bytes available)
static constexpr uint8_t REG_FIFO_STATUS1  = 0xFE;  // overflow/underflow flags
static constexpr uint8_t REG_FIFO_DATA     = 0xFF;
static constexpr uint8_t FIFO_STATUS1_OVERFLOW  = (1 << 7);
static constexpr uint8_t FIFO_STATUS1_UNDERFLOW = (1 << 6);

#endif  // TSL2585_REGISTERS_H

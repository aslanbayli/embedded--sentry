#include <mbed.h>
#include "gyro.h"

// SPI interface to gyro (using different naming)
SPI spiGyro(PF_9, PF_8, PF_7); // MOSI, MISO, SCLK lines for gyro
DigitalOut chipSelect(PC_1);   // Chip select for gyro

// Calibration parameters
int16_t xCalibThreshold; 
int16_t yCalibThreshold; 
int16_t zCalibThreshold;

// Zero-offset reference samples
int16_t xZeroSample; 
int16_t yZeroSample; 
int16_t zZeroSample;

// Sensitivity factor (dynamic based on config)
float angularSensitivity = 0.0f;

// Raw data pointer for global usage
GyrRawData *globalRawDataPtr;

// Send a byte+data over SPI and control chip select line
static void SpiWriteByte(uint8_t addr, uint8_t val) {
    chipSelect = 0;
    spiGyro.write(addr);
    spiGyro.write(val);
    chipSelect = 1;
}

// Read raw gyro registers into a structure
// This routine assumes continuous read mode with auto-incremented address
static void ReadGyroRawData(GyrRawData *rawPtr) {
    chipSelect = 0;
    spiGyro.write((OUT_X_L | 0x80 | 0x40));
    int16_t xLow = spiGyro.write(0xff);
    int16_t xHigh = spiGyro.write(0xff);
    int16_t yLow = spiGyro.write(0xff);
    int16_t yHigh = spiGyro.write(0xff);
    int16_t zLow = spiGyro.write(0xff);
    int16_t zHigh = spiGyro.write(0xff);
    chipSelect = 1;

    rawPtr->x_raw = (xHigh << 8) | xLow;
    rawPtr->y_raw = (yHigh << 8) | yLow;
    rawPtr->z_raw = (zHigh << 8) | zLow;
}

// Perform a gyro calibration routine to establish zero-level and thresholds
// This measures multiple samples and sets "no movement" reference points
static void CalibrateGyro(GyrRawData *rawPtr) {
    int16_t accumX = 0;
    int16_t accumY = 0;
    int16_t accumZ = 0;

    // Measure multiple readings to find stable zero offsets and thresholds
    for (int idx = 0; idx < 128; idx++) {
        ReadGyroRawData(rawPtr);
        accumX += rawPtr->x_raw;
        accumY += rawPtr->y_raw;
        accumZ += rawPtr->z_raw;

        if (abs(rawPtr->x_raw) > abs(xCalibThreshold)) { xCalibThreshold = rawPtr->x_raw; }
        if (abs(rawPtr->y_raw) > abs(yCalibThreshold)) { yCalibThreshold = rawPtr->y_raw; }
        if (abs(rawPtr->z_raw) > abs(zCalibThreshold)) { zCalibThreshold = rawPtr->z_raw; }

        // Wait between samples
        wait_us(10000);
    }

    // Compute average zero offsets
    xZeroSample = accumX >> 7; // division by 128 (2^7)
    yZeroSample = accumY >> 7;
    zZeroSample = accumZ >> 7;
}

// Initialize gyro: configure SPI, sensitivity, power and run calibration
void SetupGyroscope(Gyroscope_Init_Parameters *config, GyrRawData *rawInitPtr) {
    globalRawDataPtr = rawInitPtr;

    chipSelect = 1;
    // Set SPI format and frequency according to spec
    spiGyro.format(8, 3);     
    spiGyro.frequency(1000000); 

    // Write configuration registers
    SpiWriteByte(CTRL_REG_1, (config->conf1 | POWERON)); // Power on and axes enabling
    SpiWriteByte(CTRL_REG_3, config->conf3);             // DRDY enable if needed
    SpiWriteByte(CTRL_REG_4, config->conf4);             // Set full-scale selection

    // Determine sensitivity based on full-scale selection
    switch (config->conf4) {
        case FULL_SCALE_245:
            angularSensitivity = SENSITIVITY_245;
            break;
        case FULL_SCALE_500:
            angularSensitivity = SENSITIVITY_500;
            break;
        case FULL_SCALE_2000:
        case FULL_SCALE_2000_ALT:
            angularSensitivity = SENSITIVITY_2000;
            break;
        default:
            angularSensitivity = SENSITIVITY_245; // fallback
            break;
    }

    // Run calibration after setup
    CalibrateGyro(globalRawDataPtr);
}

// Convert a raw axis value to degrees per second using current sensitivity
float RawToDps(int16_t rawVal) {
    return (float)rawVal * angularSensitivity;
}

// Convert a raw axis value directly to linear velocity (assuming known constants)
float DpsToVelocity(int16_t rawVal) {
    // Combine raw val, sensitivity, conversion from degrees to radians, and a predefined scale
    float velocity = (float)rawVal * angularSensitivity * DEGREE_TO_RAD * MY_LEG;
    return velocity;
}

// Compute total traveled distance from a set of raw samples 
// (assuming each sample is taken at intervals that allow a 0.05f time step)
float ComputeDistance(int16_t sampleArray[]) {
    float totalDist = 0.0f;
    for (int i = 0; i < 400; i++) {
        float instantaneousVel = DpsToVelocity(sampleArray[i]);
        totalDist += fabsf(instantaneousVel * 0.05f);
    }
    return totalDist;
}

// Acquire fresh gyro data, remove offset, and apply threshold to minimize noise
void UpdateCalibratedData() {
    ReadGyroRawData(globalRawDataPtr);

    // Remove zero-level offsets
    globalRawDataPtr->x_raw -= xZeroSample;
    globalRawDataPtr->y_raw -= yZeroSample;
    globalRawDataPtr->z_raw -= zZeroSample;

    // Apply thresholds: if below threshold, set to zero
    if (abs(globalRawDataPtr->x_raw) < abs(xCalibThreshold)) { globalRawDataPtr->x_raw = 0; }
    if (abs(globalRawDataPtr->y_raw) < abs(yCalibThreshold)) { globalRawDataPtr->y_raw = 0; }
    if (abs(globalRawDataPtr->z_raw) < abs(zCalibThreshold)) { globalRawDataPtr->z_raw = 0; }
}

// Shut down the gyro by writing zero to control register 1
void DeactivateGyro() {
    SpiWriteByte(CTRL_REG_1, 0x00);
}

#include <mbed.h>

// Initialization parameters for the gyroscope
typedef struct {
    uint8_t conf1; // Output data rate configuration
    uint8_t conf3; // Interrupt / DRDY configuration
    uint8_t conf4; // Full-scale range selection
} GyroInitParams;

// Raw gyro data
typedef struct {
    int16_t x_raw; 
    int16_t y_raw;
    int16_t z_raw;
} GyrRawData;

// Calibrated gyro data
typedef struct {
    int16_t x_calibrated;
    int16_t y_calibrated;
    int16_t z_calibrated;
} GyrCalibratedData;

// SPI write operation
void SpiWriteByte(uint8_t addr, uint8_t val);

// Acquire raw data directly from the gyro
void ReadGyroRawData(GyrRawData *rawPtr);

// Calibrate the gyro to determine offsets and thresholds
void CalibrateGyro(GyrRawData *rawPtr);

// Configure and initialize the gyro hardware
void SetupGyroscope(GyroInitParams *config, GyrRawData *rawInitPtr);

// Convert raw data units to degrees per second
float RawToDps(int16_t rawVal);

// Convert degrees per second to linear velocity (m/s)
float DpsToVelocity(int16_t rawVal);

// Compute total traveled distance based on raw samples
float ComputeDistance(int16_t sampleArray[]);

// Acquire fresh data, apply calibration offsets/thresholds
void UpdateCalibratedData();

// Power down the gyroscope
void DeactivateGyro();

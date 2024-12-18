#include <mbed.h>

// Initialization parameters
typedef struct
{
    uint8_t conf1;       // output data rate
    uint8_t conf3;       // interrupt configuration
    uint8_t conf4;       // full sacle selection
} Gyroscope_Init_Parameters;

// Raw data
typedef struct
{
    int16_t x_raw; // X-axis raw data
    int16_t y_raw; // Y-axis raw data
    int16_t z_raw; // Z-axis raw data
} Gyroscope_RawData;

// Calibrated data
typedef struct
{
    int16_t x_calibrated; // X-axis calibrated data
    int16_t y_calibrated; // Y-axis calibrated data
    int16_t z_calibrated; // Z-axis calibrated data
} Gyroscope_CalibratedData;

// Write IO
void WriteByte(uint8_t address, uint8_t data);

// Read IO
void GetGyroValue(Gyroscope_RawData *rawdata);

// Gyroscope calibration
void CalibrateGyroscope(Gyroscope_RawData *rawdata);

// Gyroscope initialization
void InitiateGyroscope(Gyroscope_Init_Parameters *init_parameters, Gyroscope_RawData *init_raw_data);

// Data conversion: raw -> dps
float ConvertToDPS(int16_t rawdata);

// Data conversion: dps -> m/s
float ConvertToVelocity(int16_t rawdata);

// Calculate distance from raw data array;
float GetDistance(int16_t arr[]);

// Get calibrated data
void GetCalibratedRawData();

// Turn off the gyroscope
void PowerOff();
// Initialization parameters for rotation sensor
typedef struct
{
    uint8_t sampling_rate_conf; // configure the sampling rate
    uint8_t irq_conf;           // configure interrupt settings
    uint8_t scale_conf;         // configure full-scale range
} RotationSensor_Init_Params;

// Raw sensor output data
typedef struct
{
    int16_t x_axis_value; // Raw data on the X-axis
    int16_t y_axis_value; // Raw data on the Y-axis
    int16_t z_axis_value; // Raw data on the Z-axis
} RotationSensor_RawValues;

// Calibrated sensor data
typedef struct
{
    int16_t x_axis_cal; // Calibrated X-axis data
    int16_t y_axis_cal; // Calibrated Y-axis data
    int16_t z_axis_cal; // Calibrated Z-axis data
} RotationSensor_CalibratedValues;

// Write a single byte to the sensor
void Transmitter_WriteByte(uint8_t address, uint8_t data);

// Retrieve raw rotation data from the sensor
void RetrieveRotationData(RotationSensor_RawValues *rawdata);

// Execute a calibration routine on the rotation sensor
void CalibrateRotationSensor(RotationSensor_RawValues *rawdata);

// Initialize the rotation sensor with given parameters
void InitializeRotationSensor(RotationSensor_Init_Params *init_parameters, RotationSensor_RawValues *init_raw_data);

// Convert raw data to degrees per second
float RawToDPS(int16_t rawdata);

// Convert degrees per second to linear velocity (m/s)
float DPSToLinearVelocity(int16_t rawdata);

// Compute total travel distance from an array of raw values
float ComputeTravelDistance(int16_t arr[]);

// Retrieve calibrated rotation data
void FetchCalibratedRotationData();

// Turn off the rotation sensor
void DeactivateSensor();

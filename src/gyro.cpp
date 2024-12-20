#include <mbed.h>
#include "gyro.h" // changed from "gyro.h" to match the updated header
#include "constants.h"

SPI rotation_sensor_spi(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs_line(PC_1);

// Axis calibration thresholds
int16_t x_axis_threshold; 
int16_t y_axis_threshold; 
int16_t z_axis_threshold; 

// Axis zero-rate samples
int16_t x_axis_sample; 
int16_t y_axis_sample; 
int16_t z_axis_sample; 

float sensitivity = 0.0f;

RotationSensor_RawValues *rotation_values; // pointer to hold raw sensor data

// Write a single byte to the rotation sensor
void Transmitter_WriteByte(uint8_t address, uint8_t data)
{
  cs_line = 0;
  rotation_sensor_spi.write(address);
  rotation_sensor_spi.write(data);
  cs_line = 1;
}

// Retrieve raw rotation data from the sensor
void RetrieveRotationData(RotationSensor_RawValues *rawdata)
{
  cs_line = 0;
  // Using renamed register for X-axis low byte, preserving SPI read flags
  rotation_sensor_spi.write(X_AXIS_LOW_DATA_REG | 0x80 | 0x40); // auto-increment read
  rawdata->x_axis_value = rotation_sensor_spi.write(0xff) | (rotation_sensor_spi.write(0xff) << 8);
  rawdata->y_axis_value = rotation_sensor_spi.write(0xff) | (rotation_sensor_spi.write(0xff) << 8);
  rawdata->z_axis_value = rotation_sensor_spi.write(0xff) | (rotation_sensor_spi.write(0xff) << 8);
  cs_line = 1;
}

// Execute a calibration routine on the rotation sensor
void CalibrateRotationSensor(RotationSensor_RawValues *rawdata)
{
  int16_t sumX = 0;
  int16_t sumY = 0;
  int16_t sumZ = 0;
  printf("========[Calibrating rotation sensor...]========\r\n");
  for (int i = 0; i < 128; i++)
  {
    RetrieveRotationData(rawdata);
    sumX += rawdata->x_axis_value;
    sumY += rawdata->y_axis_value;
    sumZ += rawdata->z_axis_value;
    x_axis_threshold = max(x_axis_threshold, rawdata->x_axis_value);
    y_axis_threshold = max(y_axis_threshold, rawdata->y_axis_value);
    z_axis_threshold = max(z_axis_threshold, rawdata->z_axis_value);
    wait_us(10000);
  }

  x_axis_sample = sumX >> 7; // 128 is 2^7
  y_axis_sample = sumY >> 7;
  z_axis_sample = sumZ >> 7;
  printf("========[Calibration complete.]========\r\n");
}

// Initialize the rotation sensor with given parameters
void InitializeRotationSensor(RotationSensor_Init_Params *init_parameters, RotationSensor_RawValues *init_raw_data)
{
  printf("\r\n========[Initializing rotation sensor...]========\r\n");
  rotation_values = init_raw_data;
  cs_line = 1;
  // set up rotation sensor
  rotation_sensor_spi.format(8, 3);       // 8 bits per SPI frame; polarity 1, phase 0
  rotation_sensor_spi.frequency(1000000); // 1 MHz SPI clock frequency

  // Configure sensor registers using the updated structure fields
  Transmitter_WriteByte(ODR_BW_CTRL_REG, init_parameters->sampling_rate_conf | DEVICE_POWER_ON); 
  Transmitter_WriteByte(INTERRUPT_CTRL_REG, init_parameters->irq_conf);                      
  Transmitter_WriteByte(DATA_FORMAT_CTRL_REG, init_parameters->scale_conf);                  

  switch (init_parameters->scale_conf)
  {
    case FULL_SCALE_245_DPS:
      sensitivity = SENSITIVITY_245_DPS_PER_DIGIT;
      break;

    case FULL_SCALE_500_DPS:
      sensitivity = SENSITIVITY_500_DPS_PER_DIGIT;
      break;

    case FULL_SCALE_2000_DPS:
      sensitivity = SENSITIVITY_2000_DPS_PER_DIGIT;
      break;

    case FULL_SCALE_2000_DPS_ALT:
      sensitivity = SENSITIVITY_2000_DPS_PER_DIGIT;
      break;
  }

  CalibrateRotationSensor(rotation_values); 
  printf("========[Initialization complete.]========\r\n");
}

// Convert raw data to degrees per second
float RawToDPS(int16_t axis_data)
{
  float dps = axis_data * sensitivity;
  return dps;
}

// Convert degrees per second to linear velocity (m/s)
float DPSToLinearVelocity(int16_t axis_data)
{
  float velocity = axis_data * sensitivity * DEGREES_TO_RADIANS * MOUNT_POSITION;
  return velocity;
}

// Compute total travel distance from an array of raw values
float ComputeTravelDistance(int16_t arr[])
{
  float distance = 0.00f;
  for (int i = 0; i < 400; i++)
  {
    float v = DPSToLinearVelocity(arr[i]);
    distance += fabsf(v * 0.05f);
  }
  return distance;
}

// Retrieve calibrated rotation data
void FetchCalibratedRotationData()
{
  RetrieveRotationData(rotation_values);

  // Offset the zero rate level
  rotation_values->x_axis_value -= x_axis_sample;
  rotation_values->y_axis_value -= y_axis_sample;
  rotation_values->z_axis_value -= z_axis_sample;

  // Apply threshold filtering
  if (abs(rotation_values->x_axis_value) < abs(x_axis_threshold))
    rotation_values->x_axis_value = 0;
  if (abs(rotation_values->y_axis_value) < abs(y_axis_threshold))
    rotation_values->y_axis_value = 0;
  if (abs(rotation_values->z_axis_value) < abs(z_axis_threshold))
    rotation_values->z_axis_value = 0;
}

// Turn off the rotation sensor
void DeactivateSensor()
{
  Transmitter_WriteByte(ODR_BW_CTRL_REG, 0x00);
}

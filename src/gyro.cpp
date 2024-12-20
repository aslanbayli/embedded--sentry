#include <mbed.h>
#include "gyro.h"
#include "constants.h"

SPI gyroscope(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);

int16_t x_threshold; // X-axis calibration threshold
int16_t y_threshold; // Y-axis calibration threshold
int16_t z_threshold; // Z-axis calibration threshold

int16_t x_sample; // X-axis zero-rate level sample
int16_t y_sample; // Y-axis zero-rate level sample
int16_t z_sample; // Z-axis zero-rate level sample

float sensitivity = 0.0f;

Gyroscope_RawData *gyro_raw;

// Write I/O
void WriteByte(uint8_t address, uint8_t data)
{
  cs = 0;
  gyroscope.write(address);
  gyroscope.write(data);
  cs = 1;
}

// Get raw data from gyroscope
void GetGyroValue(Gyroscope_RawData *rawdata)
{
  cs = 0;
  // Using renamed register for X-axis low byte, and preserving SPI read flags
  gyroscope.write(X_AXIS_LOW_DATA_REG | 0x80 | 0x40); // auto-incremented read
  rawdata->x_raw = gyroscope.write(0xff) | (gyroscope.write(0xff) << 8);
  rawdata->y_raw = gyroscope.write(0xff) | (gyroscope.write(0xff) << 8);
  rawdata->z_raw = gyroscope.write(0xff) | (gyroscope.write(0xff) << 8);
  cs = 1;
}

// Calibrate gyroscope before recording
void CalibrateGyroscope(Gyroscope_RawData *rawdata)
{
  int16_t sumX = 0;
  int16_t sumY = 0;
  int16_t sumZ = 0;
  printf("========[Calibrating...]========\r\n");
  for (int i = 0; i < 128; i++)
  {
    GetGyroValue(rawdata);
    sumX += rawdata->x_raw;
    sumY += rawdata->y_raw;
    sumZ += rawdata->z_raw;
    x_threshold = max(x_threshold, rawdata->x_raw);
    y_threshold = max(y_threshold, rawdata->y_raw);
    z_threshold = max(z_threshold, rawdata->z_raw);
    wait_us(10000);
  }

  x_sample = sumX >> 7; // 128 is 2^7
  y_sample = sumY >> 7;
  z_sample = sumZ >> 7;
  printf("========[Calibration finish.]========\r\n");
}

// Initiate gyroscope, set up control registers
void InitiateGyroscope(Gyroscope_Init_Parameters *init_parameters, Gyroscope_RawData *init_raw_data)
{
  printf("\r\n========[Initializing gyroscope...]========\r\n");
  gyro_raw = init_raw_data;
  cs = 1;
  // set up gyroscope
  gyroscope.format(8, 3);       // 8 bits per SPI frame; polarity 1, phase 0
  gyroscope.frequency(1000000); // 1 MHz SPI clock frequency

  // Replace old register names with the newly defined ones
  WriteByte(ODR_BW_CTRL_REG, init_parameters->conf1 | DEVICE_POWER_ON); // set ODR, enable axes
  WriteByte(INTERRUPT_CTRL_REG, init_parameters->conf3);                // DRDY enable
  WriteByte(DATA_FORMAT_CTRL_REG, init_parameters->conf4);              // Full-scale range, data format

  switch (init_parameters->conf4)
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

  CalibrateGyroscope(gyro_raw); 
  printf("========[Initiation finish.]========\r\n");
}

// convert raw data to dps
float ConvertToDPS(int16_t axis_data)
{
  float dps = axis_data * sensitivity;
  return dps;
}

// convert dps to linear velocity
float ConvertToVelocity(int16_t axis_data)
{
  // Replace old constants with new ones
  float velocity = axis_data * sensitivity * DEGREES_TO_RADIANS * MOUNT_POSITION_LEG;
  return velocity;
}

// Calculate distance from raw data array
float GetDistance(int16_t arr[])
{
  float distance = 0.00f;
  for (int i = 0; i < 400; i++)
  {
    float v = ConvertToVelocity(arr[i]);
    distance += fabsf(v * 0.05f);
  }
  return distance;
}

// convert raw data to calibrated data directly
void GetCalibratedRawData()
{
  GetGyroValue(gyro_raw);

  // offset the zero rate level
  gyro_raw->x_raw -= x_sample;
  gyro_raw->y_raw -= y_sample;
  gyro_raw->z_raw -= z_sample;

  // apply threshold filtering
  if (abs(gyro_raw->x_raw) < abs(x_threshold))
    gyro_raw->x_raw = 0;
  if (abs(gyro_raw->y_raw) < abs(y_threshold))
    gyro_raw->y_raw = 0;
  if (abs(gyro_raw->z_raw) < abs(z_threshold))
    gyro_raw->z_raw = 0;
}

// turn off the gyroscope
void PowerOff()
{
  WriteByte(ODR_BW_CTRL_REG, 0x00);
}

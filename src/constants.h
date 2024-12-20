// Device identification register
#define DEVICE_ID_REG          0x0F // Holds the gyroscope's device ID

// Control registers for configuration
#define ODR_BW_CTRL_REG        0x20 // Controls output data rate and bandwidth
#define HIGH_PASS_FILTER_CTRL_REG 0x21 // Configures high-pass filter parameters
#define INTERRUPT_CTRL_REG     0x22 // Enables and configures interrupt generation
#define DATA_FORMAT_CTRL_REG   0x23 // Sets data format, including full-scale range
#define ADV_FEATURES_CTRL_REG  0x24 // Additional features control (e.g., filter enable)

// Status register
#define DATA_STATUS_REG        0x27 // Indicates when new data is available

// Output data registers for angular velocity (X, Y, Z axes)
#define X_AXIS_LOW_DATA_REG    0x28 // Low byte of X-axis angular velocity data
#define X_AXIS_HIGH_DATA_REG   0x29 // High byte of X-axis angular velocity data
#define Y_AXIS_LOW_DATA_REG    0x2A // Low byte of Y-axis angular velocity data
#define Y_AXIS_HIGH_DATA_REG   0x2B // High byte of Y-axis angular velocity data
#define Z_AXIS_LOW_DATA_REG    0x2C // Low byte of Z-axis angular velocity data
#define Z_AXIS_HIGH_DATA_REG   0x2D // High byte of Z-axis angular velocity data

// FIFO configuration and status registers
#define FIFO_CTRL_CONFIG_REG   0x2E // Configures FIFO mode and threshold
#define FIFO_STATUS_REG        0x2F // Reflects FIFO current fill level and status

// Interrupt configuration registers
#define INT1_CONFIG_REG        0x30 // Configures conditions for INT1 generation
#define INT1_SOURCE_REG        0x31 // Indicates which interrupt event has occurred
#define INT1_X_THRESHOLD_HIGH_REG 0x32 // X-axis high-threshold (INT1)
#define INT1_X_THRESHOLD_LOW_REG  0x33 // X-axis low-threshold (INT1)
#define INT1_Y_THRESHOLD_HIGH_REG 0x34 // Y-axis high-threshold (INT1)
#define INT1_Y_THRESHOLD_LOW_REG  0x35 // Y-axis low-threshold (INT1)
#define INT1_Z_THRESHOLD_HIGH_REG 0x36 // Z-axis high-threshold (INT1)
#define INT1_Z_THRESHOLD_LOW_REG  0x37 // Z-axis low-threshold (INT1)
#define INT1_DURATION_REG          0x38 // Duration for which threshold must be met to trigger INT1

// Output Data Rate (ODR) and cutoff frequency selections
#define ODR_100HZ_CUTOFF_12_5HZ  0x00
#define ODR_100HZ_CUTOFF_25HZ    0x10
#define ODR_200HZ_CUTOFF_12_5HZ  0x40
#define ODR_200HZ_CUTOFF_25HZ    0x50
#define ODR_200HZ_CUTOFF_50HZ    0x60
#define ODR_200HZ_CUTOFF_70HZ    0x70
#define ODR_400HZ_CUTOFF_20HZ    0x80
#define ODR_400HZ_CUTOFF_25HZ    0x90
#define ODR_400HZ_CUTOFF_50HZ    0xA0
#define ODR_400HZ_CUTOFF_110HZ   0xB0
#define ODR_800HZ_CUTOFF_30HZ    0xC0
#define ODR_800HZ_CUTOFF_35HZ    0xD0
#define ODR_800HZ_CUTOFF_50HZ    0xE0
#define ODR_800HZ_CUTOFF_110HZ   0xF0

// High-pass filter settings (currently disabled)
#define HPF_DISABLED_100HZ       0x00
#define HPF_DISABLED_200HZ       0x00
#define HPF_DISABLED_400HZ       0x00
#define HPF_DISABLED_800HZ       0x00

// Interrupt configuration bits
#define INT1_PIN_ENABLE         0x80 // Enable INT1 pin output
#define INT1_BOOT_STATUS_ENABLE 0x40 // Boot status routed to INT1 pin
#define INT1_ACTIVE_CONFIG      0x20 // Active level configuration for INT1
#define INT1_OPEN_DRAIN_MODE    0x10 // INT1 pin set to open-drain mode
#define INT1_LATCH_INTERRUPT    0x02 // Interrupt remains latched until source is read

// Axis-specific interrupt event enables for INT1
#define INT1_Z_HIGH_ENABLE      0x20 // Trigger on Z-axis high threshold event
#define INT1_Z_LOW_ENABLE       0x10 // Trigger on Z-axis low threshold event
#define INT1_Y_HIGH_ENABLE      0x08 // Trigger on Y-axis high threshold event
#define INT1_Y_LOW_ENABLE       0x04 // Trigger on Y-axis low threshold event
#define INT1_X_HIGH_ENABLE      0x02 // Trigger on X-axis high threshold event
#define INT1_X_LOW_ENABLE       0x01 // Trigger on X-axis low threshold event

// INT2 pin configurations
#define INT2_DATA_READY         0x08 // Data-ready signal on DRDY/INT2 pin

// Full-scale range selections
#define FULL_SCALE_245_DPS      0x00 // ±245 dps
#define FULL_SCALE_500_DPS      0x10 // ±500 dps
#define FULL_SCALE_2000_DPS     0x20 // ±2000 dps (primary)
#define FULL_SCALE_2000_DPS_ALT 0x30 // ±2000 dps (alternate configuration)

// Sensitivities in dps/digit
#define SENSITIVITY_245_DPS_PER_DIGIT  0.00875f
#define SENSITIVITY_500_DPS_PER_DIGIT  0.0175f
#define SENSITIVITY_2000_DPS_PER_DIGIT 0.07f

// Conversion and placement parameters
#define MOUNT_POSITION_LEG       1       // Device mounted on leg, approximately 0.8m above ground
#define DEGREES_TO_RADIANS       0.0175f // Conversion factor for degrees to radians (π/180 ≈ 0.017453)

// Power management states
#define DEVICE_POWER_ON          0x0F // Power on the gyroscope
#define DEVICE_POWER_OFF         0x00 // Power off the gyroscope

// Sampling time and intervals
#define SAMPLE_TIME_MS_20        20      // Sample time of 20 ms
#define SAMPLE_INTERVAL_S_0_05   0.005f  // Sample interval of 0.05 seconds

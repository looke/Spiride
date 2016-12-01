const byte MPU9250_SPI_READ_MASK = 0x80;

const byte MPU9250_REG_WHO_AM_I = 0x75;


const byte MPU9250_I2C_MAG_ADDRESS = 0x0C;

// Power Management
const byte MPU9250_REG_PWR_MGMT_1 = 0x6B;    // RW
const byte MPU9250_REG_PWR_MGMT_2 = 0x6C;    // RW

// Sample Rate Divider
#define MPU9250_REG_SMPLRT_DIV          0x19    // RW

// Configuration
#define MPU9250_REG_CONFIG              0x1A    // RW

// Gyroscope configuration
#define MPU9250_REG_GYRO_CONFIG         0x1B    // RW

// Accelerometer Configuration
#define MPU9250_REG_ACCEL_CONFIG        0x1C    // RW
#define MPU9250_REG_ACCEL_CONFIG2       0x1D    // RW

// INT Pin / Bypass Enable Configuration
#define MPU9250_REG_INT_PIN_CFG         0x37    // RW
// Interrupt Enable
#define MPU9250_REG_INT_ENABLE          0x38    // RW
// Interrupt Status
#define MPU9250_REG_INT_STATUS          0x3A    // R

// User Control
#define MPU9250_REG_USER_CTRL           0x6A    // RW
// I2C Master Control
#define MPU9250_REG_I2C_MST_CTRL        0x24    // RW
// I2C Master Delay Control
#define MPU9250_REG_I2C_MST_DELAY_CTRL  0x67    // RW

// I2C Slave 0 Control
#define MPU9250_REG_I2C_SLV0_ADDR       0x25    // RW
#define MPU9250_REG_I2C_SLV0_REG        0x26    // RW
#define MPU9250_REG_I2C_SLV0_CTRL       0x27    // RW

// I2C Slave 0 Data Out
#define MPU9250_REG_I2C_SLV0_DO         0x63    // RW

// Accelerometer Measurements
#define MPU9250_REG_ACCEL_XOUT_H        0x3B    // R
#define MPU9250_REG_ACCEL_XOUT_L        0x3C    // R
#define MPU9250_REG_ACCEL_YOUT_H        0x3D    // R
#define MPU9250_REG_ACCEL_YOUT_L        0x3E    // R
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F    // R
#define MPU9250_REG_ACCEL_ZOUT_L        0x40    // R

// Temperature Measurement
#define MPU9250_REG_TEMP_OUT_H          0x41    // R
#define MPU9250_REG_TEMP_OUT_L          0x42    // R

// Gyroscope Measurements
#define MPU9250_REG_GYRO_XOUT_H         0x43    // R
#define MPU9250_REG_GYRO_XOUT_L         0x44    // R
#define MPU9250_REG_GYRO_YOUT_H         0x45    // R
#define MPU9250_REG_GYRO_YOUT_L         0x46    // R
#define MPU9250_REG_GYRO_ZOUT_H         0x47    // R
#define MPU9250_REG_GYRO_ZOUT_L         0x48    // R

// External Sensor Data
#define MPU9250_REG_EXT_SENS_DATA_00    0x49    // R
#define MPU9250_REG_EXT_SENS_DATA_01    0x4A    // R
#define MPU9250_REG_EXT_SENS_DATA_02    0x4B    // R
#define MPU9250_REG_EXT_SENS_DATA_03    0x4C    // R
#define MPU9250_REG_EXT_SENS_DATA_04    0x4D    // R
#define MPU9250_REG_EXT_SENS_DATA_05    0x4E    // R
#define MPU9250_REG_EXT_SENS_DATA_06    0x4F    // R
#define MPU9250_REG_EXT_SENS_DATA_07    0x50    // R
#define MPU9250_REG_EXT_SENS_DATA_08    0x51    // R
#define MPU9250_REG_EXT_SENS_DATA_09    0x52    // R
#define MPU9250_REG_EXT_SENS_DATA_10    0x53    // R
#define MPU9250_REG_EXT_SENS_DATA_11    0x54    // R
#define MPU9250_REG_EXT_SENS_DATA_12    0x55    // R
#define MPU9250_REG_EXT_SENS_DATA_13    0x56    // R
#define MPU9250_REG_EXT_SENS_DATA_14    0x57    // R
#define MPU9250_REG_EXT_SENS_DATA_15    0x58    // R
#define MPU9250_REG_EXT_SENS_DATA_16    0x59    // R
#define MPU9250_REG_EXT_SENS_DATA_17    0x5A    // R
#define MPU9250_REG_EXT_SENS_DATA_18    0x5B    // R
#define MPU9250_REG_EXT_SENS_DATA_19    0x5C    // R
#define MPU9250_REG_EXT_SENS_DATA_20    0x5D    // R
#define MPU9250_REG_EXT_SENS_DATA_21    0x5E    // R
#define MPU9250_REG_EXT_SENS_DATA_22    0x5F    // R
#define MPU9250_REG_EXT_SENS_DATA_23    0x60    // R


/*Mag Reg*/
// Device ID (Who Am I)
#define MPU9250_MAG_REG_WAI             0x00    // R
// Information for AKM
#define MPU9250_MAG_REG_INFO            0x01    // R
// Status 1
#define MPU9250_MAG_REG_ST1             0x02    // R
// Measurement Data
#define MPU9250_MAG_REG_HXL             0x03    // R
#define MPU9250_MAG_REG_HXH             0x04    // R
#define MPU9250_MAG_REG_HYL             0x05    // R
#define MPU9250_MAG_REG_HYH             0x06    // R
#define MPU9250_MAG_REG_HZL             0x07    // R
#define MPU9250_MAG_REG_HZH             0x08    // R
// Status 2
#define MPU9250_MAG_REG_ST2             0x09    // R
// Control 1
#define MPU9250_MAG_REG_CNTL1            0x0A    // RW
// Control 2
#define MPU9250_MAG_REG_CNTL2            0x0B    // RW
// Self test control
#define MPU9250_MAG_REG_ASTC            0x0C    // RW
// Disable I2C
#define MPU9250_MAG_REG_I2CDIS          0x0F    // RW
// Sensitivity adjustment values
// Hadj = H * ((((ASA - 128)*0.5)/128) + 1)
#define MPU9250_MAG_REG_ASAX            0x10    // R
#define MPU9250_MAG_REG_ASAY            0x11    // R
#define MPU9250_MAG_REG_ASAZ            0x12    // R

#define MPU9250_MAG_WAI_VALUE           0x48    // Use for connection test
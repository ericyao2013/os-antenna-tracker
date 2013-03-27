/******************************************************************************/
/******************         Board Definitions         *************************/
/******************************************************************************/
#if defined(HK_MW_PRO)

  #define GYRO
  #define ITG3205
  #define GYRO_I2C_SPEED 400hz
  #define ITG3200_ADDRESS 0X68
  
  #define ACCEL//Accel
  #define BMA180
  #define BMA180_ADDRESS 0x40  

  //#define BARO //Barometer
  //#define BMP085

  #define MAG //Magnometer
  #define HMC5883L
  #define HMC5883_ADDRESS 0x1E
  
  //Sensor orientation
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  = -Z;}
#endif

/******************************************************************************/
/******************         Sensor Definitions        *************************/
/******************************************************************************/

//HMC5883 Magnometer
#if defined(HMC5883) || defined(HMC5883L)
  #define HMC5883
  
  #define HMC5883_DATA_REGISTER 0x03
  
  #define HMC5883_GAIN_88       0x00         // +/- 0.88 Ga
  #define HMC5883_GAIN_88_SCALE 0.73
  #define HMC5883_GAIN_13       0x20         // +/- 1.3 Ga (default)
  #define HMC5883_GAIN_13_SCALE 0.92  
  #define HMC5883_GAIN_19       0x40         // +/- 1.9 Ga
  #define HMC5883_GAIN_25       0x60         // +/- 2.5 Ga
  #define HMC5883_GAIN_40       0x80         // +/- 4.0 Ga
  #define HMC5883_GAIN_47       0xA0         // +/- 4.7 Ga
  #define HMC5883_GAIN_56       0xC0         // +/- 5.6 Ga
  #define HMC5883_GAIN_81       0xE0         // +/- 8.1 Ga
  
  #define HMC5883_R_CONFA 0x00
  #define HMC5883_R_CONFB 0x01
  #define HMC5883_R_MODE 0x02
  #define HMC5883_X_SELF_TEST_GAUSS (+1.16)   //!< X axis level when bias current is applied.
  #define HMC5883_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
  #define HMC5883_Z_SELF_TEST_GAUSS (+1.08)   //!< Y axis level when bias current is applied.
  #define SELF_TEST_LOW_LIMIT  (243.0/390.0)  //!< Low limit when gain is 5.
  #define SELF_TEST_HIGH_LIMIT (575.0/390.0)  //!< High limit when gain is 5.
  #define HMC_POS_BIAS 1
  #define HMC_NEG_BIAS 2
  
#endif //HMC5883


//ITG3200 or ITG3205 Triple Axis Gyros - both have essentially the same configuration options
#if defined(ITG3200) || defined(ITG205)
  #define ITG3200
  #define ITG3200_REGISTER_SIZE 8
  
  /* ---- Registers ---- */
  #define WHO_AM_I           0x00  // RW   SETUP: I2C address  
  #define SMPLRT_DIV         0x15  // RW   SETUP: Sample Rate Divider      
  #define DLPF_FS            0x16  // RW   SETUP: Digital Low Pass Filter/ Full Scale range
  #define INT_CFG            0x17  // RW   Interrupt: Configuration
  #define INT_STATUS         0x1A  // R   Interrupt: Status
  #define TEMP_OUT           0x1B  // R   SENSOR: Temperature 2bytes
  #define GYRO_XOUT          0x1D  // R   SENSOR: Gyro X 2bytes  
  #define GYRO_YOUT          0x1F  // R   SENSOR: Gyro Y 2bytes
  #define GYRO_ZOUT          0x21  // R   SENSOR: Gyro Z 2bytes
  #define PWR_MGM            0x3E  // RW  Power Management
  
  /* ---- bit maps ---- */
  #define DLPFFS_FS_SEL             0x18  // 00011000
  #define DLPFFS_DLPF_CFG           0x07  // 00000111
  #define INTCFG_ACTL               0x80  // 10000000
  #define INTCFG_OPEN               0x40  // 01000000
  #define INTCFG_LATCH_INT_EN       0x20  // 00100000
  #define INTCFG_INT_ANYRD_2CLEAR   0x10  // 00010000
  #define INTCFG_ITG_RDY_EN         0x04  // 00000100
  #define INTCFG_RAW_RDY_EN         0x01  // 00000001
  #define INTSTATUS_ITG_RDY         0x04  // 00000100
  #define INTSTATUS_RAW_DATA_RDY    0x01  // 00000001
  #define PWRMGM_HRESET             0x80  // 10000000
  #define PWRMGM_SLEEP              0x40  // 01000000
  #define PWRMGM_STBY_XG            0x20  // 00100000
  #define PWRMGM_STBY_YG            0x10  // 00010000
  #define PWRMGM_STBY_ZG            0x08  // 00001000
  #define PWRMGM_CLK_SEL            0x07  // 00000111
  
  /************************************/
  /*    REGISTERS PARAMETERS    */
  /************************************/
  // Sample Rate Divider
  #define NOSRDIVIDER         0 // default    FsampleHz=SampleRateHz/(divider+1)
  // Gyro Full Scale Range
  #define RANGE2000           3   // default
  // Digital Low Pass Filter BandWidth and SampleRate
  #define BW256_SR8           0   // default    256Khz BW and 8Khz SR
  #define BW188_SR1           1
  #define BW098_SR1           2
  #define BW042_SR1           3
  #define BW020_SR1           4
  #define BW010_SR1           5
  #define BW005_SR1           6
  // Interrupt Active logic lvl
  #define ACTIVE_ONHIGH       0 // default
  #define ACTIVE_ONLOW        1
  // Interrupt drive type
  #define PUSH_PULL           0 // default
  #define OPEN_DRAIN          1
  // Interrupt Latch mode
  #define PULSE_50US          0 // default
  #define UNTIL_INT_CLEARED   1
  // Interrupt Latch clear method
  #define READ_STATUSREG      0 // default
  #define READ_ANYREG         1
  // Power management
  #define NORMAL              0 // default
  #define STANDBY             1
  // Clock Source - user parameters
  #define INTERNALOSC         0   // default
  #define PLL_XGYRO_REF       1
  #define PLL_YGYRO_REF       2
  #define PLL_ZGYRO_REF       3
  #define PLL_EXTERNAL32      4   // 32.768 kHz
  #define PLL_EXTERNAL19      5   // 19.2 Mhz

#endif

//BMA180 Accelerometer
#if defined(BMA180)
  #define BMA180
  #define BMA180_RESET  0x10
  #define BMA180_REGISTER_SIZE 8
  #define BMA180_PWR 0x0D
  #define BMA180_BW 0X20
  #define BMA180_RANGE 0X35
  #define BMA_180_DATA 0x02
#endif

//BMP085 Barometer
#if defined(BMP085)
  #define BMP085
  #define BMP085_ADDRESS 0x77
#endif

/******************************************************************************/
/******************      EEPROM Storage Addresses     *************************/
/******************************************************************************/
// Do not change these or you will have problems
// 2 bytes per value
/******************************************************************************/

// MAG Min and Max calibration values
#define MAG_MIN_X_ADDR 0
#define MAG_MAX_X_ADDR 2
#define MAG_MIN_Y_ADDR 4
#define MAG_MAX_Y_ADDR 6
#define MAG_MIN_Z_ADDR 8
#define MAG_MAX_Z_ADDR 10

// ACCEL LEVEL Values
#define ACCEL_OFFSET_X_ADDR 12
#define ACCEL_OFFSET_Y_ADDR 14
#define ACCEL_OFFSET_Z_ADDR 16

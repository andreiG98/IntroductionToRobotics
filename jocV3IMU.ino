#include <Wire.h>
#include "LedControl.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>

#define V0_PIN 9
#define BUZZER A3
#define INPUT_BUTTON A1

// The name of the sensor is "MPU-6050".
// For program code, I omit the '-', 
// therefor I use the name "MPU6050....".


// Register names according to the datasheet.
// According to the InvenSense document 
// "MPU-6000 and MPU-6050 Register Map 
// and Descriptions Revision 3.2", there are no registers
// at 0x02 ... 0x18, but according other information 
// the registers in that unknown area are for gain 
// and offsets.
// 
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68


// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of 
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte 
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally, 
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

float angle_x;
float angle_y;

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value
  
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
   
  int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

// The sensor should be motionless on a horizontal surface 
//  while calibration is happening
void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
  
  //Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  //Serial.println("Finishing Calibration");
}

int buttonPressed = 0;
int IMUMoved = 0;

LedControl lc = LedControl(12, 11, 10, 1); //DIN, CLK, LOAD, No.Driver
LiquidCrystal lcd(8, 3, 4, 5, 6, 7);

bool matrixDisplay[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

byte sad[8] = {
  B00111100,
  B01000010,
  B10100101,
  B10010001,
  B10010001,
  B10100101,
  B01000010,
  B00111100
};

byte one[8] = {
  B00000000,
  B00000000,
  B10000000,
  B11111111,
  B10000010,
  B00000100,
  B00000000,
  B00000000
};

byte two[8] = {
  B00000000,
  B00000000,
  B10000111,
  B10001001,
  B10010001,
  B11100010,
  B00000000,
  B00000000
};

byte three[8] = {
  B00000000,
  B00000000,
  B11110111,
  B10001001,
  B10001001,
  B10000001,
  B00000000,
  B00000000
};

struct highScore {
  int incepator;
  int avansat;
  int freestyle;
};

int liniePunct = random(7), coloanaPunct = random(7);
int linieCorp, coloanaCorp;
int prevLiniePunct, prevColoanaPunct;
int prevLinieCorp, prevColoanaCorp;
int score;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void buzzer(){
  //tone(BUZZER, 1000);
  //delay(1000);
  //noTone(BUZZER);
}

int mode = 1;

void meniu(){
  int buttonValue = analogRead(A1);
  Serial.println(buttonValue);
  int period = 1500;
  unsigned long timeNow = millis();

  lc.clearDisplay(0);
  for (int i = 0; i < 8; i++)
  {
    lc.setColumn(0, i, one[i]);
  }
  
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Welcome to");
  lcd.setCursor(4, 1);
  lcd.print("my game!"); 
  while(millis() < timeNow + period);
  
  timeNow = millis();
  lcd.clear();
  lcd.print("1 - Beginner");
  while(millis() < timeNow + period);
  
  timeNow = millis();
  lcd.clear();
  lcd.print("2 - Advanced");
  while(millis() < timeNow + period);
  
  timeNow = millis();
  lcd.clear();
  lcd.print("3 - Freestyle");
  while(millis() < timeNow + period);
  lcd.clear();

  while(buttonValue <= 100 || buttonValue >= 940){
    buttonValue = analogRead(A1);
    Serial.println(buttonValue);
  
    if(buttonValue >= 965 && buttonPressed == 0){
      buttonPressed = 1;
      switch(mode){
        case 1:
          mode = 2;
          lc.clearDisplay(0);
          for (int i = 0; i < 8; i++)
          {
            lc.setColumn(0, i, two[i]);
          }
          break;
        case 2:
          mode = 3;
          lc.clearDisplay(0);
          for (int i = 0; i < 8; i++)
          {
            lc.setColumn(0, i, three[i]);
          }
          break;
        case 3:  
          mode = 1;
          lc.clearDisplay(0);
          for (int i = 0; i < 8; i++)
          {
            lc.setColumn(0, i, one[i]);
          }
          break;
      }  
    }
    if(buttonValue >=0 && buttonValue <= 100)
      buttonPressed = 0;
  }

  do {
    IMURead();
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Loading...");
  } while(angle_x > -49 || angle_x < -54 || angle_y > -24 || angle_y < -28);
    
  switch(mode){
    case 1:
      nivelIncepator();
      break;
    case 2:
      nivelAvansat();
      break;
    case 3:  
      nivelFreestyle();
      break;
  }  
}

void printMatrix(){
  //lc.clearDisplay(0);
  int i, j;
  for(i = 0; i < 8; i++){
    for(j = 0; j < 8; j++){
      lc.setLed(0, i, j, matrixDisplay[i][j]);
    }
  }
}

void printMatrixAdvanced(int linieCorp, int coloanaCorp){
  lc.clearDisplay(0);
  int i, j;
  int period = 2000;
  unsigned long timeNow = 0;
  for(i = 0; i < 8; i++){
    for(j = 0; j < 8; j++){
      lc.setLed(0, i, j, matrixDisplay[i][j]);
      timeNow = millis();
      //while(millis() < timeNow + period);
      if(i == linieCorp && j == coloanaCorp && timeNow + period > millis()){
        matrixDisplay[i][j] = 1 - matrixDisplay[i][j];
        //lc.setLed(0, i, j, matrixDisplay[i][j]);
        //matrixDisplay[i][j] = 1;
      }
    }
  }
}

void timeUP(){
  lc.clearDisplay(0);
  for (int i = 0; i < 8; i++)
  {
    lc.setColumn(0, i, sad[i]);
  }
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Time is up");
  lcd.setCursor(1, 1);
  lcd.print("Your score: ");
  lcd.setCursor(13, 1);
  lcd.print(score);
  buzzer();
  delay(3000);  //vreau sa-mi ramana scrisul pe lcd
  highScore auxScore;
  EEPROM.get(0, auxScore);
  switch(mode){
    case 1:
      if(score > auxScore.incepator){
        auxScore.incepator = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
    case 2:
      if(score > auxScore.avansat){
        auxScore.avansat = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
    case 3:  
      if(score > auxScore.freestyle){
        auxScore.freestyle = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
  }
  delay(3000);  //vreau sa-mi ramana scrisul pe lcd
  EEPROM.put(0, auxScore);
  resetFunc();  //call reset
}

void IMURead(){
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  // Read the raw values.
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
  
  // Get the time of reading for rotation computations
  unsigned long t_now = millis();
   
  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet: 
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
  
  
  // Get raw acceleration values
  //float G_CONVERT = 16384;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;
  
  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;

  float accel_angle_z = 0;
  
  // Compute the (filtered) gyro angles
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
}

void nivelFreestyle(){
  
  randomSeed(analogRead(A0)); 
  liniePunct = random(7);
  coloanaPunct = random(7);
  linieCorp = random(7);
  coloanaCorp = random(7);
  
  int timeNow = millis();
  long timpMaxim = 45000;
  int i, j;
  while(timpMaxim > 0) {
    IMURead();
    if(millis() - timeNow >= 1000){
      timpMaxim -= 1000;
      timeNow += 1000;
      if(timpMaxim == 0)
        timeUP();  
    }
    matrixDisplay[liniePunct][coloanaPunct] = 1;
    for(i = linieCorp; i <= linieCorp + 1; i++){
        for(j = coloanaCorp; j <= coloanaCorp + 1; j++){
            matrixDisplay[i][j] = 1;
        }
    }
    printMatrix();
    
    prevLiniePunct = liniePunct;
    prevColoanaPunct = coloanaPunct;

    Serial.print(angle_x);
    Serial.print(F(","));
    Serial.print(angle_y);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if(angle_x > -50 && angle_y < -30){
      IMUMoved = 1;
      coloanaPunct++;
      if(coloanaPunct >=8)
        coloanaPunct = 0;  
    }
    if(angle_x < -60 && angle_y > -25){
      IMUMoved = 1;
      coloanaPunct--;
      if(coloanaPunct < 0)
        coloanaPunct = 7;  
    }

    if(angle_x < -40 && angle_y < -40){
      IMUMoved = 1;
      liniePunct++;
      if(liniePunct >= 8)
        liniePunct = 0;  
    }

    if(angle_x < -40 && angle_y > -10){
      IMUMoved = 1;
      liniePunct--;
      if(liniePunct < 0)
        liniePunct = 7;  
    }
  
    if(liniePunct == linieCorp && coloanaPunct == coloanaCorp || liniePunct == linieCorp && coloanaPunct == coloanaCorp + 1 || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp + 1){
        score++;
        prevLinieCorp = linieCorp;
        prevColoanaCorp = coloanaCorp;
        linieCorp = random(7);
        coloanaCorp = random(7);  
    }

    lcd.clear();
    lcd.home();
    lcd.print("Score: ");
    lcd.setCursor(7, 0);
    lcd.print(score);
    lcd.setCursor(0, 1);
    lcd.print("Timer: ");
    lcd.setCursor(7, 1);
    lcd.print(timpMaxim/1000);
    
    matrixDisplay[prevLiniePunct][prevColoanaPunct] = 0;
    for(i = prevLinieCorp; i <= prevLinieCorp + 1; i++){
        for(j = prevColoanaCorp; j <= prevColoanaCorp + 1; j++){
            matrixDisplay[i][j] = 0;
        }
    }
  }
}

void nivelAvansat(){
  randomSeed(analogRead(A0)); 
  liniePunct = random(7);
  coloanaPunct = random(7);
  linieCorp = random(7);
  coloanaCorp = random(7);
  
  int timeNow = millis();
  int timpMaxim = 30000;
  int i, j;
  while(timpMaxim > 0){
    IMURead();
    if(millis() - timeNow >= 1000){
      timpMaxim -= 1000;
      timeNow += 1000;
      if(timpMaxim == 0)
        timeUP();  
    }
    matrixDisplay[liniePunct][coloanaPunct] = 1;
    matrixDisplay[linieCorp][coloanaCorp] = 1;
    
    //printMatrixAdvanced(linieCorp, coloanaCorp);
    printMatrix();
    
    prevLiniePunct = liniePunct;
    prevColoanaPunct = coloanaPunct;

    Serial.print(angle_x);
    Serial.print(F(","));
    Serial.print(angle_y);
    Serial.print(F(","));
    Serial.println("");
    delay(5);
  
    if(angle_x > -40 && angle_y < -30){
      coloanaPunct++;
      if(coloanaPunct >=8){
        coloanaPunct = 0;
        score -= 2;
      }
        //dead();
    }
    if(angle_x < -60 && angle_y > -25){
      coloanaPunct--;
      if(coloanaPunct < 0){
        coloanaPunct = 7;
        score -= 2;
      }
        //dead();  
    }

    if(angle_x < -40 && angle_y < -40){
      liniePunct++;
      if(liniePunct >= 8){
        liniePunct = 0;
        score -= 2;
      }
        //dead();  
    }

    if(angle_x < -40 && angle_y > -10){
      liniePunct--;
      if(liniePunct < 0){
        liniePunct = 7;
        score -= 2;
      }
        //dead();  
    }
  
    if(liniePunct == linieCorp && coloanaPunct == coloanaCorp){
        score += 10;
        prevLinieCorp = linieCorp;
        prevColoanaCorp = coloanaCorp;
        linieCorp = random(7);
        coloanaCorp = random(7);  
        matrixDisplay[prevLinieCorp][prevColoanaCorp] = 0;  
    }

    lcd.clear();
    lcd.home();
    lcd.print("Score: ");
    Serial.println(score);
    lcd.setCursor(7, 0);
    lcd.print(score);
    lcd.setCursor(0, 1);
    lcd.print("Timer: ");
    lcd.setCursor(7, 1);
    lcd.print(timpMaxim/1000);
    matrixDisplay[prevLiniePunct][prevColoanaPunct] = 0;
  }
}

void nivelIncepator(){
  randomSeed(analogRead(A0)); 
  liniePunct = random(7);
  coloanaPunct = random(7);
  linieCorp = random(7);
  coloanaCorp = random(7);
  
  int timeNow = millis();
  int timpMaxim = 30000;
  int i, j;
  while(timpMaxim > 0){
    IMURead();
    if(millis() - timeNow >= 1000){
      timpMaxim -= 1000;
      timeNow += 1000;
      if(timpMaxim == 0)
        timeUP();  
    }
    matrixDisplay[liniePunct][coloanaPunct] = 1;
    for(i = linieCorp; i <= linieCorp + 1; i++){
        for(j = coloanaCorp; j <= coloanaCorp + 1; j++){
            matrixDisplay[i][j] = 1;
        }
    }
    printMatrix();
    
    prevLiniePunct = liniePunct;
    prevColoanaPunct = coloanaPunct;

    Serial.print(angle_x);
    Serial.print(F(","));
    Serial.print(angle_y);
    Serial.print(F(","));
    Serial.println("");
    delay(5);
  
    if(angle_x > -40 && angle_y < -30){
      coloanaPunct++;
      if(coloanaPunct >=8) {
        coloanaPunct = 0;
        score--;  
      }
        //dead();
    }
    if(angle_x < -60 && angle_y > -25){
      coloanaPunct--;
      if(coloanaPunct < 0) {
          coloanaPunct = 7;
          score--;
      }
        //dead();  
    }

    if(angle_x < -40 && angle_y < -40){
      liniePunct++;
      if(liniePunct >= 8) {
        liniePunct = 0;
        score--;  
      }
        //dead();  
    }

    if(angle_x < -40 && angle_y > -10){
      liniePunct--;
      if(liniePunct < 0) {
        liniePunct = 7;  
        score--;
      }
        //dead();  
    }
  
    if(liniePunct == linieCorp && coloanaPunct == coloanaCorp || liniePunct == linieCorp && coloanaPunct == coloanaCorp + 1 || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp + 1){
        score += 10;
        prevLinieCorp = linieCorp;
        prevColoanaCorp = coloanaCorp;
        linieCorp = random(7);
        coloanaCorp = random(7);  
    }

    lcd.clear();
    lcd.home();
    lcd.print("Score: ");
    Serial.println(score);
    lcd.setCursor(7, 0);
    lcd.print(score);
    lcd.setCursor(0, 1);
    lcd.print("Timer: ");
    lcd.setCursor(7, 1);
    lcd.print(timpMaxim/1000);
    
    matrixDisplay[prevLiniePunct][prevColoanaPunct] = 0;
    for(i = prevLinieCorp; i <= prevLinieCorp + 1; i++){
        for(j = prevColoanaCorp; j <= prevColoanaCorp + 1; j++){
            matrixDisplay[i][j] = 0;
        }
    }
  }
}

void setup() {    
  lc.shutdown(0, false); //turn off power saving, enable display
  lc.setIntensity(0, 0); //sets brightness
  lc.clearDisplay(0); //clear screen
  
  lcd.begin(16,2); //seteaza dim
  pinMode(V0_PIN, OUTPUT);
  analogWrite(V0_PIN, 100);

  pinMode(INPUT_BUTTON, INPUT);

  pinMode(BUZZER, OUTPUT);

  score = 0;
  
  int error;
  uint8_t c;

  Serial.begin(19200);
  
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);

  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  
  //Initialize the angles
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}

void loop() {
  //IMURead();
  //nivelFreestyle();
  //meniu();
  nivelIncepator();
  //nivelAvansat();
}

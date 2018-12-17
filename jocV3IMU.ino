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

#define MPU6050_ACCEL_XOUtH       0x3B   // R  
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
typedef union accelTGyroUnion {
  struct {
    uint8_t xAccelH;
    uint8_t xAccelL;
    uint8_t yAccelH;
    uint8_t yAccelL;
    uint8_t tH;
    uint8_t tL;
    uint8_t xGyroH;
    uint8_t xGyroL;
    uint8_t yGyroH;
    uint8_t yGyroL;
  } reg;
  struct {
    int xAccel;
    int yAccel;
    int zAccel;
    int xGyro;
    int yGyro;
    int zGyro;
  } value;
};

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long lastReadTime;
float         lastXAngle;  // These are the filtered angles
float         lastYAngle;
float         lastZAngle;
float         lastGyroXAngle;  // Store the gyro angles to compare drift
float         lastGyroYAngle;
float         lastGyroZAngle;

void setLastReadAngleData(unsigned long time, float x, float y, float z, float xGyro, float yGyro, float zGyro) {
  lastReadTime = time;
  lastXAngle = x;
  lastYAngle = y;
  lastZAngle = z;
  lastGyroXAngle = xGyro;
  lastGyroYAngle = yGyro;
  lastGyroZAngle = zGyro;
}

inline unsigned long getLastTime() { return lastReadTime; }
inline float getlastXAngle() { return lastXAngle; }
inline float getlastYAngle() { return lastYAngle; }
inline float getlastZAngle() { return lastZAngle; }
inline float getlastGyroXAngle() { return lastGyroXAngle; }
inline float getlastGyroYAngle() { return lastGyroYAngle; }
inline float getlastGyroZAngle() { return lastGyroZAngle; }

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float baseXAccel;
float baseYAccel;
float baseZAccel;

float baseXGyro;
float baseYGyro;
float baseZGyro;

float angleX;
float angleY;

int buttonPressed = 0;
int IMUMoved = 0;
int liniePunct;
int coloanaPunct;
int linieCorp;
int coloanaCorp;
int prevLiniePunct;
int prevColoanaPunct;
int prevLinieCorp;
int prevColoanaCorp;
int score;
int mode = 1;

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

LedControl lc = LedControl(12, 11, 10, 1); //DIN, CLK, LOAD, No.Driver
LiquidCrystal lcd(8, 3, 4, 5, 6, 7);

// --------------------------------------------------------
// MPU6050Read
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
int MPU6050Read(int start, uint8_t *buffer, int size) {
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
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050Write
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
//   MPU6050Write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050Write(int start, const uint8_t *pData, int size) {
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
// MPU6050WriteReg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050WriteReg(int reg, uint8_t data) {
  int error;

  error = MPU6050Write(reg, &data, 1);

  return (error);
}

int readGyroAccelVals(uint8_t* accelTGyroPtr) {
  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value

  accelTGyroUnion* accelTGyro = (accelTGyroUnion *) accelTGyroPtr;

  int error = MPU6050Read (MPU6050_ACCEL_XOUtH, (uint8_t *) accelTGyro, sizeof(*accelTGyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like xAccelL does no
  // longer contain the lower byte.
  uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accelTGyro).reg.xAccelH, (*accelTGyro).reg.xAccelL);
  SWAP ((*accelTGyro).reg.yAccelH, (*accelTGyro).reg.yAccelL);
  SWAP ((*accelTGyro).reg.tH, (*accelTGyro).reg.tL);
  SWAP ((*accelTGyro).reg.xGyroH, (*accelTGyro).reg.xGyroL);
  SWAP ((*accelTGyro).reg.yGyroL, (*accelTGyro).reg.yGyroL);

  return error;
}

// The sensor should be motionless on a horizontal surface
//  while calibration is happening
void calibrateSensors() {
  int                   numReadings = 10;
  float                 xAccel = 0;
  float                 yAccel = 0;
  float                 zAccel = 0;
  float                 xGyro = 0;
  float                 yGyro = 0;
  float                 zGyro = 0;
  accelTGyroUnion    accelTGyro;

  //Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  readGyroAccelVals((uint8_t *) &accelTGyro);

  // Read and average the raw values from the IMU
  for (int i = 0; i < numReadings; i++) {
    readGyroAccelVals((uint8_t *) &accelTGyro);
    xAccel += accelTGyro.value.xAccel;
    yAccel += accelTGyro.value.yAccel;
    zAccel += accelTGyro.value.zAccel;
    xGyro += accelTGyro.value.xGyro;
    yGyro += accelTGyro.value.yGyro;
    zGyro += accelTGyro.value.zGyro;
    delay(100);
  }
  xAccel /= numReadings;
  yAccel /= numReadings;
  zAccel /= numReadings;
  xGyro /= numReadings;
  yGyro /= numReadings;
  zGyro /= numReadings;

  // Store the raw calibration values globally
  baseXAccel = xAccel;
  baseYAccel = yAccel;
  baseZAccel = zAccel;
  baseXGyro = xGyro;
  baseYGyro = yGyro;
  baseZGyro = zGyro;

  //Serial.println("Finishing Calibration");
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void buzzer() {
  //tone(BUZZER, 1000);
  //delay(1000);
  //noTone(BUZZER);
}

void meniu() {
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
  delay(period);

  timeNow = millis();
  lcd.clear();
  lcd.print("1 - Beginner");
  delay(period);

  timeNow = millis();
  lcd.clear();
  lcd.print("2 - Advanced");
  delay(period);

  timeNow = millis();
  lcd.clear();
  lcd.print("3 - Freestyle");
  delay(period);
  lcd.clear();

  while (buttonValue <= 200 || buttonValue >= 900) {
    buttonValue = analogRead(A1);
    Serial.println(buttonValue);

    if (buttonValue > 900 && buttonPressed == 0) {
      buttonPressed = 1;
      switch (mode) {
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
    if (buttonValue >= 0 && buttonValue <= 100)
      buttonPressed = 0;
  }

  do {
    IMURead();
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Loading...");
  } while (angleX > -49 || angleX < -54 || angleY > -24 || angleY < -28);

  switch (mode) {
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

void printMatrix() {
  //lc.clearDisplay(0);
  int i, j;
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      lc.setLed(0, i, j, matrixDisplay[i][j]);
    }
  }
}

/*void printMatrixAdvanced(int linieCorp, int coloanaCorp) {
  lc.clearDisplay(0);
  int i, j;
  int period = 2000;
  unsigned long timeNow = 0;
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      lc.setLed(0, i, j, matrixDisplay[i][j]);
      timeNow = millis();
      //while(millis() < timeNow + period);
      if (i == linieCorp && j == coloanaCorp && timeNow + period > millis()) {
        matrixDisplay[i][j] = 1 - matrixDisplay[i][j];
        //lc.setLed(0, i, j, matrixDisplay[i][j]);
        //matrixDisplay[i][j] = 1;
      }
    }
  }
}*/

void timeUP() {
  int buttonValue;
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
  switch (mode) {
    case 1:
      if (score > auxScore.incepator) {
        auxScore.incepator = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
    case 2:
      if (score > auxScore.avansat) {
        auxScore.avansat = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
    case 3:
      if (score > auxScore.freestyle) {
        auxScore.freestyle = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
  }
  delay(2000);  //vreau sa-mi ramana scrisul pe lcd
  EEPROM.put(0, auxScore);
  lcd.clear();
  do {
    buttonValue = analogRead(A1);
    Serial.println(buttonValue);
    lcd.setCursor(0, 0);
    lcd.print("Press any button");
    lcd.setCursor(2, 1);
    lcd.print("to try again");
  } while (buttonValue < 100);
  resetFunc();  //call reset
}

void IMURead() {
  int error;
  double dT;
  accelTGyroUnion accelTGyro;

  // Read the raw values.
  error = readGyroAccelVals((uint8_t*) &accelTGyro);

  // Get the time of reading for rotation computations
  unsigned long timeNow = millis();

  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet:
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  float gyro_x = (accelTGyro.value.xGyro - baseXGyro) / FS_SEL;
  float gyro_y = (accelTGyro.value.yGyro - baseYGyro) / FS_SEL;
  float gyro_z = (accelTGyro.value.zGyro - baseZGyro) / FS_SEL;


  // Get raw acceleration values
  //float G_CONVERT = 16384;
  float accel_x = accelTGyro.value.xAccel;
  float accel_y = accelTGyro.value.yAccel;
  float accel_z = accelTGyro.value.zAccel;

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180 / 3.14159;
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angleY = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
  float accel_angleX = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;

  float accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  float dt = (timeNow - getLastTime()) / 1000.0;
  float gyro_angleX = gyro_x * dt + getlastXAngle();
  float gyro_angleY = gyro_y * dt + getlastYAngle();
  float gyro_angle_z = gyro_z * dt + getlastZAngle();

  // Compute the drifting gyro angles
  float unfiltered_gyro_angleX = gyro_x * dt + getlastGyroXAngle();
  float unfiltered_gyro_angleY = gyro_y * dt + getlastGyroYAngle();
  float unfiltered_gyro_angle_z = gyro_z * dt + getlastGyroZAngle();

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  angleX = alpha * gyro_angleX + (1.0 - alpha) * accel_angleX;
  angleY = alpha * gyro_angleY + (1.0 - alpha) * accel_angleY;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  setLastReadAngleData(timeNow, angleX, angleY, angle_z, unfiltered_gyro_angleX, unfiltered_gyro_angleY, unfiltered_gyro_angle_z);
}

void nivelFreestyle() {

  randomSeed(analogRead(A0));
  liniePunct = random(7);
  coloanaPunct = random(7);
  linieCorp = random(7);
  coloanaCorp = random(7);

  int timeNow = millis();
  long timpMaxim = 45000;
  int i, j;
  while (timpMaxim > 0) {
    IMURead();
    if (millis() - timeNow >= 1000) {
      timpMaxim -= 1000;
      timeNow += 1000;
      if (timpMaxim == 0)
        timeUP();
    }
    matrixDisplay[liniePunct][coloanaPunct] = 1;
    for (i = linieCorp; i <= linieCorp + 1; i++) {
      for (j = coloanaCorp; j <= coloanaCorp + 1; j++) {
        matrixDisplay[i][j] = 1;
      }
    }
    printMatrix();

    prevLiniePunct = liniePunct;
    prevColoanaPunct = coloanaPunct;

    Serial.print(angleX);
    Serial.print(F(","));
    Serial.print(angleY);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if (angleX > -50 && angleY < -30) {
      IMUMoved = 1;
      coloanaPunct++;
      if (coloanaPunct >= 8)
        coloanaPunct = 0;
    }
    if (angleX < -60 && angleY > -25) {
      IMUMoved = 1;
      coloanaPunct--;
      if (coloanaPunct < 0)
        coloanaPunct = 7;
    }

    if (angleX < -40 && angleY < -40) {
      IMUMoved = 1;
      liniePunct++;
      if (liniePunct >= 8)
        liniePunct = 0;
    }

    if (angleX < -40 && angleY > -10) {
      IMUMoved = 1;
      liniePunct--;
      if (liniePunct < 0)
        liniePunct = 7;
    }

    if (liniePunct == linieCorp && coloanaPunct == coloanaCorp || liniePunct == linieCorp && coloanaPunct == coloanaCorp + 1 || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp + 1) {
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
    lcd.print(timpMaxim / 1000);

    matrixDisplay[prevLiniePunct][prevColoanaPunct] = 0;
    for (i = prevLinieCorp; i <= prevLinieCorp + 1; i++) {
      for (j = prevColoanaCorp; j <= prevColoanaCorp + 1; j++) {
        matrixDisplay[i][j] = 0;
      }
    }
  }
}

void nivelAvansat() {
  randomSeed(analogRead(A0));
  liniePunct = random(7);
  coloanaPunct = random(7);
  linieCorp = random(7);
  coloanaCorp = random(7);

  int timeNow = millis();
  int timpMaxim = 30000;
  int i, j;
  while (timpMaxim > 0) {
    IMURead();
    if (millis() - timeNow >= 1000) {
      timpMaxim -= 1000;
      timeNow += 1000;
      if (timpMaxim == 0)
        timeUP();
    }
    matrixDisplay[liniePunct][coloanaPunct] = 1;
    matrixDisplay[linieCorp][coloanaCorp] = 1;

    //printMatrixAdvanced(linieCorp, coloanaCorp);
    printMatrix();

    prevLiniePunct = liniePunct;
    prevColoanaPunct = coloanaPunct;

    Serial.print(angleX);
    Serial.print(F(","));
    Serial.print(angleY);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if (angleX > -40 && angleY < -30) {
      coloanaPunct++;
      if (coloanaPunct >= 8) {
        coloanaPunct = 0;
        score -= 2;
      }
      //dead();
    }
    if (angleX < -60 && angleY > -25) {
      coloanaPunct--;
      if (coloanaPunct < 0) {
        coloanaPunct = 7;
        score -= 2;
      }
      //dead();
    }

    if (angleX < -40 && angleY < -40) {
      liniePunct++;
      if (liniePunct >= 8) {
        liniePunct = 0;
        score -= 2;
      }
      //dead();
    }

    if (angleX < -40 && angleY > -10) {
      liniePunct--;
      if (liniePunct < 0) {
        liniePunct = 7;
        score -= 2;
      }
      //dead();
    }

    if (liniePunct == linieCorp && coloanaPunct == coloanaCorp) {
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
    lcd.print(timpMaxim / 1000);
    matrixDisplay[prevLiniePunct][prevColoanaPunct] = 0;
  }
}

void nivelIncepator() {
  randomSeed(analogRead(A0));
  liniePunct = random(7);
  coloanaPunct = random(7);
  linieCorp = random(7);
  coloanaCorp = random(7);

  int timeNow = millis();
  int timpMaxim = 30000;
  int i, j;
  while (timpMaxim > 0) {
    IMURead();
    if (millis() - timeNow >= 1000) {
      timpMaxim -= 1000;
      timeNow += 1000;
      if (timpMaxim == 0)
        timeUP();
    }
    matrixDisplay[liniePunct][coloanaPunct] = 1;
    for (i = linieCorp; i <= linieCorp + 1; i++) {
      for (j = coloanaCorp; j <= coloanaCorp + 1; j++) {
        matrixDisplay[i][j] = 1;
      }
    }
    printMatrix();

    prevLiniePunct = liniePunct;
    prevColoanaPunct = coloanaPunct;

    Serial.print(angleX);
    Serial.print(F(","));
    Serial.print(angleY);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if (angleX > -40 && angleY < -30) {
      coloanaPunct++;
      if (coloanaPunct >= 8) {
        coloanaPunct = 0;
        score--;
      }
      //dead();
    }
    if (angleX < -60 && angleY > -25) {
      coloanaPunct--;
      if (coloanaPunct < 0) {
        coloanaPunct = 7;
        score--;
      }
      //dead();
    }

    if (angleX < -40 && angleY < -40) {
      liniePunct++;
      if (liniePunct >= 8) {
        liniePunct = 0;
        score--;
      }
      //dead();
    }

    if (angleX < -40 && angleY > -10) {
      liniePunct--;
      if (liniePunct < 0) {
        liniePunct = 7;
        score--;
      }
      //dead();
    }

    if (liniePunct == linieCorp && coloanaPunct == coloanaCorp || liniePunct == linieCorp && coloanaPunct == coloanaCorp + 1 || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp || liniePunct == linieCorp + 1 && coloanaPunct == coloanaCorp + 1) {
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
    lcd.print(timpMaxim / 1000);

    matrixDisplay[prevLiniePunct][prevColoanaPunct] = 0;
    for (i = prevLinieCorp; i <= prevLinieCorp + 1; i++) {
      for (j = prevColoanaCorp; j <= prevColoanaCorp + 1; j++) {
        matrixDisplay[i][j] = 0;
      }
    }
  }
}

void setup() {
  int error;
  uint8_t c;
  Serial.begin(19200);

  lc.shutdown(0, false); //turn off power saving, enable display
  lc.setIntensity(0, 0); //sets brightness
  lc.clearDisplay(0); //clear screen

  lcd.begin(16, 2); //seteaza dim
  pinMode(V0_PIN, OUTPUT);
  analogWrite(V0_PIN, 100);

  pinMode(INPUT_BUTTON, INPUT);

  pinMode(BUZZER, OUTPUT);

  score = 0;

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050Read (MPU6050_WHO_AM_I, &c, 1);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050Read (MPU6050_PWR_MGMT_2, &c, 1);

  // Clear the 'sleep' bit to start the sensor.
  MPU6050WriteReg (MPU6050_PWR_MGMT_1, 0);

  //Initialize the angles
  calibrateSensors();
  setLastReadAngleData(millis(), 0, 0, 0, 0, 0, 0);
}

void loop() {
  //IMURead();
  //nivelFreestyle();
  meniu();
  //nivelIncepator();
  //nivelAvansat();
  //timeUP();
}

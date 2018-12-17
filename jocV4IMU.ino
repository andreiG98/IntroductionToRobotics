#include <Wire.h>
#include "LedControl.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>

#define V0_PIN 9
#define BUZZER A3
#define INPUT_BUTTON A1

LedControl lc = LedControl(12, 11, 10, 1); //DIN, CLK, LOAD, No.Driver
LiquidCrystal lcd(8, 3, 4, 5, 6, 7);

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

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

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
}

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

  while (buttonValue <= 200 || buttonValue >= 950) {
    buttonValue = analogRead(A1);
    Serial.println(buttonValue);

    if (buttonValue > 950 && buttonPressed == 0) {
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

  /*do {
    //IMURead();
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Loading...");
  } while (rotX > -49 || rotX < -54 || rotY > -24 || rotY < -28);*/

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
  } while (buttonValue < 200);
  resetFunc();  //call reset
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
    recordGyroRegisters();
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

    Serial.print(rotX);
    Serial.print(F(","));
    Serial.print(rotY);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if (rotX > 50) {
      coloanaPunct++;
      if (coloanaPunct >= 8)
        coloanaPunct = 0;
    }
    if (rotX < -50) {
      coloanaPunct--;
      if (coloanaPunct < 0)
        coloanaPunct = 7;
    }

    if (rotY < -50) {
      liniePunct++;
      if (liniePunct >= 8)
        liniePunct = 0;
    }

    if (rotY > 50) {
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
    recordGyroRegisters();
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

    Serial.print(rotX);
    Serial.print(F(","));
    Serial.print(rotY);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if (rotX > 50) {
      coloanaPunct++;
      if (coloanaPunct >= 8) {
        coloanaPunct = 0; 
        score -= 2;
      }
    }
    if (rotX < -50) {
      coloanaPunct--;
      if (coloanaPunct < 0) {
        coloanaPunct = 7;
        score -= 2;
      }
    }

    if (rotY < -50) {
      liniePunct++;
      if (liniePunct >= 8) {
        liniePunct = 0;
        score -= 2;
      }
    }

    if (rotY > 50) {
      liniePunct--;
      if (liniePunct < 0) {
        liniePunct = 7;
        score -= 2;
      }
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
    recordGyroRegisters();
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

    Serial.print(rotX);
    Serial.print(F(","));
    Serial.print(rotY);
    Serial.print(F(","));
    Serial.println("");
    delay(5);

    if (rotX > 50) {
      coloanaPunct++;
      if (coloanaPunct >= 8) {
        coloanaPunct = 0;
        score--;
      }
    }
    if (rotX < -50) {
      coloanaPunct--;
      if (coloanaPunct < 0) {
        coloanaPunct = 7;
        score--;
      }
    }

    if (rotY < -50) {
      liniePunct++;
      if (liniePunct >= 8) {
        liniePunct = 0;
        score--;
      }
    }

    if (rotY > 50) {
      liniePunct--;
      if (liniePunct < 0) {
        liniePunct = 7;
        score--;
      }
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
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  
  lc.shutdown(0, false); //turn off power saving, enable display
  lc.setIntensity(0, 0); //sets brightness
  lc.clearDisplay(0); //clear screen

  lcd.begin(16, 2); //seteaza dim
  pinMode(V0_PIN, OUTPUT);
  analogWrite(V0_PIN, 100);

  pinMode(INPUT_BUTTON, INPUT);

  pinMode(BUZZER, OUTPUT);

  score = 0;
  
}

void loop() {
  //recordAccelRegisters();
  //recordGyroRegisters();
  //printData();
  //delay(100);
  meniu();
  //nivelFreestyle();
}

#include <Wire.h>
#include "LedControl.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>
#define V0_PIN 9
#define BUZZER_PIN A3
#define BUTTON_PIN A1
#define INPUT_BUTTON A1
#define RANDOM_PIN A0
#define gyroSensitivity 131
#define buzzerNoise 1000
#define periodShowLCD 1500
#define maxLineColumn 7
#define oneSecond 1000
#define maxTimeFreestyle 45000
#define maxTimeBegginerAdvanced 30000
#define rotXValueLeft 50
#define rotXValueRight -50
#define rotYValueUp -50
#define rotYValueDown 50

LedControl lc = LedControl(12, 11, 10, 1); //DIN, CLK, LOAD, No.Driver
LiquidCrystal lcd(8, 3, 4, 5, 6, 7);

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
int buttonPressed = 0;
int IMUMoved = 0;
int pointLine;
int pointColumn;
int objectLine;
int objectColumn;
int prevPointLine;
int prevPointColumn;
int prevObjectLine;
int prevObjectColumn;
int score;
int difficulty = 1;

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
  int beginner;
  int advanced;
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
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / gyroSensitivity;
  rotY = gyroY / gyroSensitivity; 
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
  tone(BUZZER_PIN, buzzerNoise);
  delay(buzzerNoise);
  noTone(BUZZER_PIN);
}

void meniu() {
  int buttonValue = analogRead(BUTTON_PIN);
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
  delay(periodShowLCD);
  timeNow = millis();
  lcd.clear();
  lcd.print("1 - Beginner");
  delay(periodShowLCD);
  timeNow = millis();
  lcd.clear();
  lcd.print("2 - Advanced");
  delay(periodShowLCD );
  timeNow = millis();
  lcd.clear();
  lcd.print("3 - Freestyle");
  delay(periodShowLCD );
  lcd.clear();
  while (buttonValue <= 200 || buttonValue >= 900) {
    buttonValue = analogRead(BUTTON_PIN);
    Serial.println(buttonValue);
    if (buttonValue > 900 && buttonPressed == 0) {
      buttonPressed = 1;
      switch (difficulty) {
        case 1:
          difficulty = 2;
          lc.clearDisplay(0);
          for (int i = 0; i < 8; i++)
          {
            lc.setColumn(0, i, two[i]);
          }
          break;
        case 2:
          difficulty = 3;
          lc.clearDisplay(0);
          for (int i = 0; i < 8; i++)
          {
            lc.setColumn(0, i, three[i]);
          }
          break;
        case 3:
          difficulty = 1;
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
  /*if(difficulty == 1)
    difficulty = 3;
  else
    difficulty--;*/
  switch (difficulty) {
    case 1:
      beginnerLevel();
      break;
    case 2:
      advancedLevel();
      break;
    case 3:
      freestyleLevel();
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
  delay(3000);
  highScore auxScore;
  EEPROM.get(0, auxScore);
  switch (difficulty) {
    case 1:
      if (score > auxScore.beginner) {
        auxScore.beginner = score;
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Congrats!");
        lcd.setCursor(3, 1);
        lcd.print("Highscore!");
      }
      break;
    case 2:
      if (score > auxScore.advanced) {
        auxScore.advanced = score;
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
    buttonValue = analogRead(BUTTON_PIN);
    lcd.setCursor(0, 0);
    lcd.print("Press any button");
    lcd.setCursor(2, 1);
    lcd.print("to try again");
  } while (buttonValue < 200);
  resetFunc();  //call reset
}

void printMatrix() {
  int i, j;
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      lc.setLed(0, i, j, matrixDisplay[i][j]);
    }
  }
}

void generatePointObject() {
  randomSeed(analogRead(RANDOM_PIN));
  pointLine = random(maxLineColumn);
  pointColumn = random(maxLineColumn);
  objectLine = random(maxLineColumn);
  objectColumn = random(maxLineColumn);
}

void freestyleLevel() {
  generatePointObject();
  unsigned long timeNow = millis();
  unsigned long maxTime = maxTimeFreestyle;
  int i, j;
  while (maxTime > 0) {
    recordGyroRegisters();
    if (millis() - timeNow >= oneSecond) {
      maxTime -= oneSecond;
      timeNow += oneSecond;
      if (maxTime == 0)
        timeUP();
    }
    matrixDisplay[pointLine][pointColumn] = 1;
    for (i = objectLine; i <= objectLine + 1; i++) {
      for (j = objectColumn; j <= objectColumn + 1; j++) {
        matrixDisplay[i][j] = 1;
      }
    }
    printMatrix();
    prevPointLine = pointLine;
    prevPointColumn = pointColumn;
    if (rotX > rotXValueLeft) {
      pointColumn++;
      if (pointColumn >= 8)
        pointColumn = 0;
    }
    if (rotX < rotXValueRight) {
      pointColumn--;
      if (pointColumn < 0)
        pointColumn = 7;
    }
    if (rotY < rotYValueUp) {
      pointLine++;
      if (pointLine >= 8)
        pointLine = 0;
    }
    if (rotY > rotYValueDown) {
      pointLine--;
      if (pointLine < 0)
        pointLine = 7;
    }
    if (pointLine == objectLine && pointColumn == objectColumn || pointLine == objectLine && pointColumn == objectColumn + 1 || pointLine == objectLine + 1 && pointColumn == objectColumn || pointLine == objectLine + 1 && pointColumn == objectColumn + 1) {
      score++;
      prevObjectLine = objectLine;
      prevObjectColumn = objectColumn;
      objectLine = random(maxLineColumn);
      objectColumn = random(maxLineColumn);
    }
    lcd.clear();
    lcd.home();
    lcd.print("Score: ");
    lcd.setCursor(7, 0);
    lcd.print(score);
    lcd.setCursor(0, 1);
    lcd.print("Timer: ");
    lcd.setCursor(7, 1);
    lcd.print(maxTime / 1000);
    matrixDisplay[prevPointLine][prevPointColumn] = 0;
    for (i = prevObjectLine; i <= prevObjectLine + 1; i++) {
      for (j = prevObjectColumn; j <= prevObjectColumn + 1; j++) {
        matrixDisplay[i][j] = 0;
      }
    }
  }
}

void advancedLevel() {
  generatePointObject();
  unsigned long timeNow = millis();
  unsigned long maxTime = maxTimeBegginerAdvanced;
  int i, j;
  while (maxTime > 0) {
    recordGyroRegisters();
    if (millis() - timeNow >= oneSecond) {
      maxTime -= oneSecond;
      timeNow += oneSecond;
      if (maxTime == 0)
        timeUP();
    }
    matrixDisplay[pointLine][pointColumn] = 1;
    matrixDisplay[objectLine][objectColumn] = 1;
    printMatrix();
    prevPointLine = pointLine;
    prevPointColumn = pointColumn;
    if (rotX > rotXValueLeft) {
      pointColumn++;
      if (pointColumn >= 8) {
        pointColumn = 0; 
        score -= 2;
      }
    }
    if (rotX < rotXValueRight) {
      pointColumn--;
      if (pointColumn < 0) {
        pointColumn = 7;
        score -= 2;
      }
    }
    if (rotY < rotYValueUp) {
      pointLine++;
      if (pointLine >= 8) {
        pointLine = 0;
        score -= 2;
      }
    }
    if (rotY > rotYValueDown) {
      pointLine--;
      if (pointLine < 0) {
        pointLine = 7;
        score -= 2;
      }
    }
    if (pointLine == objectLine && pointColumn == objectColumn) {
      score += 10;
      prevObjectLine = objectLine;
      prevObjectColumn = objectColumn;
      objectLine = random(maxLineColumn);
      objectColumn = random(maxLineColumn);
      matrixDisplay[prevObjectLine][prevObjectColumn] = 0;
    }
    lcd.clear();
    lcd.home();
    lcd.print("Score: ");
    lcd.setCursor(7, 0);
    lcd.print(score);
    lcd.setCursor(0, 1);
    lcd.print("Timer: ");
    lcd.setCursor(7, 1);
    lcd.print(maxTime / 1000);
    matrixDisplay[prevPointLine][prevPointColumn] = 0;
  }
}

void beginnerLevel() {
  generatePointObject();
  unsigned long timeNow = millis();
  unsigned long maxTime = maxTimeBegginerAdvanced;
  int i, j;
  while (maxTime > 0) {
    recordGyroRegisters();
    if (millis() - timeNow >= 1000) {
      maxTime -= oneSecond;
      timeNow += oneSecond;
      if (maxTime == 0)
        timeUP();
    }
    matrixDisplay[pointLine][pointColumn] = 1;
    for (i = objectLine; i <= objectLine + 1; i++) {
      for (j = objectColumn; j <= objectColumn + 1; j++) {
        matrixDisplay[i][j] = 1;
      }
    }
    printMatrix();
    prevPointLine = pointLine;
    prevPointColumn = pointColumn;
    if (rotX > rotXValueLeft) {
      pointColumn++;
      if (pointColumn >= 8) {
        pointColumn = 0;
        score--;
      }
    }
    if (rotX < rotXValueRight) {
      pointColumn--;
      if (pointColumn < 0) {
        pointColumn = 7;
        score--;
      }
    }
    if (rotY < rotYValueUp ) {
      pointLine++;
      if (pointLine >= 8) {
        pointLine = 0;
        score--;
      }
    }
    if (rotY > rotYValueDown) {
      pointLine--;
      if (pointLine < 0) {
        pointLine = 7;
        score--;
      }
    }
    if (pointLine == objectLine && pointColumn == objectColumn || pointLine == objectLine && pointColumn == objectColumn + 1 || pointLine == objectLine + 1 && pointColumn == objectColumn || pointLine == objectLine + 1 && pointColumn == objectColumn + 1) {
      score += 10;
      prevObjectLine = objectLine;
      prevObjectColumn = objectColumn;
      objectLine = random(maxLineColumn);
      objectColumn = random(maxLineColumn);
    }
    lcd.clear();
    lcd.home();
    lcd.print("Score: ");
    lcd.setCursor(7, 0);
    lcd.print(score);
    lcd.setCursor(0, 1);
    lcd.print("Timer: ");
    lcd.setCursor(7, 1);
    lcd.print(maxTime / 1000);
    matrixDisplay[prevPointLine][prevPointColumn] = 0;
    for (i = prevObjectLine; i <= prevObjectLine + 1; i++) {
      for (j = prevObjectColumn; j <= prevObjectColumn + 1; j++) {
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
  pinMode(BUZZER_PIN, OUTPUT);
  score = 0;
}

void loop() {
  meniu();
}

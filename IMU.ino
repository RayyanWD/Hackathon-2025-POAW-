#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> 
#include <Adafruit_ST7789.h> 
#include <SPI.h>
#include "bitmapsLarge.h"

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define CUSTOM_GRAY   0x8410 // medium gray
#define CUSTOM_PURPLE 0x780F // purple
#define CUSTOM_ORANGE 0xFD20 // orange
#define CUSTOM_GREEN  0x07E0 // green (this one is already close to ST77XX_GREEN)

const int buttonPin = 14;
int buttonState = 0;
int evenOdd = 0;

uint16_t objectColors[] = {
  CUSTOM_GRAY,    // 1: person
  CUSTOM_GREEN,   // 2: bicycle
  CUSTOM_PURPLE,  // 3: car
  CUSTOM_ORANGE   // 4: motorcycle
};


#define TFT1_CS         3
#define TFT1_DC         4

#define TFT2_CS         6  // Pick a different free pin for CS
#define TFT2_DC         7  // DC must be different if you want independent control

#define WIDTH 128
#define HEIGHT 128

Adafruit_BNO055 IMU = Adafruit_BNO055();
Adafruit_ST7735 tft = Adafruit_ST7735(TFT1_CS, TFT1_DC, 4);
Adafruit_ST7735 tft2 = Adafruit_ST7735(TFT2_CS, TFT2_DC, 7);

unsigned long lastToggleTime = 0;
float p = 3.1415926;

int counter = 0;
int mappedX = 0;
bool lastButtonState = HIGH;

bool HUDON = true;

bool pedestrianTextDisplayed = false;  // NEW VARIABLE
int detectedClassId = -1;

void setup() {
pinMode(buttonPin, INPUT_PULLUP);
Serial.begin(2000000);
IMU.begin();
delay(100);

IMU.setExtCrystalUse(true);

Serial.print(tft.width());
Serial.print(tft.height());

  tft.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab
  tft.setRotation(2); // Rotate the display upside down
  tft.fillScreen(ST77XX_BLACK);


  tft2.initR(INITR_144GREENTAB);
  tft2.setRotation(0);
  tft2.fillScreen(ST77XX_RED); // Fill second display RED to prove it's alive

}

uint16_t getColorForClass(int classId) {
  switch (classId) {
    case 1: return CUSTOM_GRAY;    // person
    case 2: return CUSTOM_GREEN;   // bicycle
    case 3: return CUSTOM_PURPLE;  // car
    case 4: return CUSTOM_ORANGE;  // motorcycle
    default: return ST77XX_BLACK;  // unknown
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(1, 15);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}



float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {

  bool pedestrianNearby = false;  // New flag to control display

  bool currentButtonState = digitalRead(buttonPin);

    if (lastButtonState == HIGH && currentButtonState == LOW) {
      counter++;
      evenOdd = counter % 2;
      Serial.println(evenOdd);
      if (evenOdd == 1) {
          tft2.fillScreen(ST77XX_BLACK);
          tft2.setCursor(45, 8);
          tft2.setTextColor(ST77XX_GREEN);
          tft2.setTextWrap(true);
          tft2.print("Recording!");
      } else {
        tft2.fillScreen(ST77XX_BLACK);
      }
    }

    lastButtonState = currentButtonState;

  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    int commaIndex = incoming.indexOf(',');

    if (commaIndex != -1) {
      // Normal case: numbers with comma
      int classId = incoming.substring(0, commaIndex).toInt();
      int width = incoming.substring(commaIndex + 1).toInt();

      detectedClassId = classId;
      pedestrianNearby = true;
      mappedX = map(width, 0, 1280, 128, 0);

    } else {
      // No comma = command string
      unsigned long currentMillis = millis();
      if (currentMillis - lastToggleTime >= 1000) { // 5 seconds = 5000 ms
        HUDON = !HUDON;
        lastToggleTime = currentMillis;

        if (!HUDON) {
          tft.fillScreen(ST77XX_BLACK);
        }

      }
    }
  }

if (HUDON) {
// Erase previous Class ID text
if (pedestrianTextDisplayed) {

  tft.fillRect(0, 1, 128, 18, ST77XX_BLACK);
  tft2.fillRect(0, 1, 128, 32, ST77XX_BLACK);

  pedestrianTextDisplayed = false;
  detectedClassId = -1;  // Reset after clearing

}

// Handle Class ID Nearby text
if (pedestrianNearby && !pedestrianTextDisplayed && detectedClassId != -1) {

  if (detectedClassId == 1 || detectedClassId == 2 || detectedClassId == 3 || detectedClassId == 4) {
  }

  if (detectedClassId == 1) {
    tft2.setCursor(14, 20);
    tft2.setTextColor(ST77XX_WHITE);
    tft2.setTextSize(1);
    tft2.print("Person nearby");
  }

  if (detectedClassId == 2) {
    tft2.setCursor(14, 20);
    tft2.setTextColor(ST77XX_WHITE);
    tft2.setTextSize(1);
    tft2.print("Bicycle nearby");
  }
  if (detectedClassId == 3) {
    tft2.setCursor(14, 20);
    tft2.setTextColor(ST77XX_WHITE);
    tft2.setTextSize(1);
    tft2.print("Car nearby");
  }
    if (detectedClassId == 4) {
    tft2.setCursor(14, 20);
    tft2.setTextColor(ST77XX_WHITE);
    tft2.setTextSize(1);
    tft2.print("Motorcycle nearby");
  }

  pedestrianTextDisplayed = true;
}


sensors_event_t event;

IMU.getEvent(&event);
  
float angleRad = event.orientation.y * (PI / 180);
float vertAngleRad = (event.orientation.y + 90) * (PI / 180);

int horizonLength = 60;
int headingLength = 20;
int markerLength = 7;

int headingMapped = -map((int(round(mapFloat(event.orientation.x + 45, 0.0, 360.0, 0.0, 480.0))) % 120), 61, 119, 0, 60);

float headingOriginX = headingMapped * -cos(angleRad);
float headingOriginY = headingMapped * -sin(angleRad);
float headingX1 = headingOriginX + 7 * cos(vertAngleRad);
float headingY1 = headingOriginY + 7 * sin(vertAngleRad);
float headingX2 = headingOriginX + headingLength * -cos(vertAngleRad);
float headingY2 = headingOriginY + headingLength * -sin(vertAngleRad);

int LLx = floor(horizonLength * -cos(angleRad));
int LLy = floor(horizonLength * -sin(angleRad));
int URx = floor(horizonLength * cos(angleRad));
int URy = floor(horizonLength * sin(angleRad));

int LLx2 = floor(12 * cos(vertAngleRad));
int LLy2 = floor(12 * sin(vertAngleRad));
int URx2 = floor(25 * cos(vertAngleRad));
int URy2 = floor(25 * sin(vertAngleRad));

float tempOrigins[9][2];
float tiltOrigins[13][2];

imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

if (acc.x() > 30 || acc.y() > 30 || acc.z() > 30) {
    int h = 128, w = 128, row, col, buffidx = 0;
    for (row = 0; row < h; row++) {
        for (col = 0; col < w; col++) {
            tft.drawPixel(col, row, pgm_read_word(evive_in_hand + buffidx));
            buffidx++;
        }
    }
    testdrawtext("Not very sigma of you", ST77XX_RED);
    delay(5000);
    tft.fillScreen(ST77XX_BLACK);
}

tft.drawLine(128 - (LLx + 64), -LLy + 64, 128 - (URx + 64), -URy + 64, ST77XX_GREEN);
tft.drawLine(128 - (LLx2 + 64), -LLy2 + 64, 128 - (URx2 + 64), -URy2 + 64, ST77XX_GREEN);
tft.drawLine(128 - (headingX1 + 64), -headingY1 + 64, 128 - (headingX2 + 64), -headingY2 + 64, ST77XX_RED);
tft.drawCircle(128 - (headingOriginX + 64), -headingOriginY + 64, 3, ST77XX_RED);

for (int i = 0, offset = 5; offset <= 85; offset += 10, i++) {
    if (offset == 45) continue;

    float tempMapped = -map((int(round(mapFloat(event.orientation.x + offset, 0.0, 360.0, 0.0, 480.0))) % 120), 61, 119, 0, 60);

    float tempOriginX = tempMapped * -cos(angleRad);
    float tempOriginY = tempMapped * -sin(angleRad);
    float tempX1 = tempOriginX + markerLength * cos(vertAngleRad);
    float tempY1 = tempOriginY + markerLength * sin(vertAngleRad);
    float tempX2 = tempOriginX + markerLength * -cos(vertAngleRad);
    float tempY2 = tempOriginY + markerLength * -sin(vertAngleRad);

    tempOrigins[i][0] = tempOriginX;
    tempOrigins[i][1] = tempOriginY;

    tft.drawLine(128 - (tempX1 + 64), -tempY1 + 64, 128 - (tempX2 + 64), -tempY2 + 64, ST77XX_RED);
}

for (int i = 0, offset = -15; offset <= 15; offset += 2.5, i++) {
    float lineLength = min(1.0 / abs(cos(angleRad)), 1.0 / abs(sin(angleRad)));

    int scaledMin = round(-1310 * (1 / lineLength));
    int scaledMax = round(1310 * (1 / lineLength));

    int tilt = round(mapFloat((event.orientation.z + offset), -90, 90, scaledMin, scaledMax));
    int tiltX = round(tilt * cos(vertAngleRad));
    int tiltY = round(tilt * sin(vertAngleRad));

    tiltOrigins[i][0] = tiltX;
    tiltOrigins[i][1] = tiltY;

    if (offset == 0) {
        tft.drawCircle(128 - (tiltX + 64), -tiltY + 64, 5, ST77XX_WHITE);
    } else {
        tft.drawCircle(128 - (tiltX + 64), -tiltY + 64, 3, ST77XX_YELLOW);
    }
}

delay(1);

for (int i = 0, offset = 5; offset <= 85; offset += 10, i++) {
    if (offset == 45) continue;

    float tempOriginX = tempOrigins[i][0];
    float tempOriginY = tempOrigins[i][1];
    float tempX1 = tempOriginX + markerLength * cos(vertAngleRad);
    float tempY1 = tempOriginY + markerLength * sin(vertAngleRad);
    float tempX2 = tempOriginX + markerLength * -cos(vertAngleRad);
    float tempY2 = tempOriginY + markerLength * -sin(vertAngleRad);
  
    tft.drawLine(128 - (tempX1 + 64), -tempY1 + 64, 128 - (tempX2 + 64), -tempY2 + 64, ST77XX_BLACK);
}

for (int i = 0, offset = -15; offset <= 15; offset += 2.5, i++) {
    int tiltX = tiltOrigins[i][0];
    int tiltY = tiltOrigins[i][1];

    if (offset == 0) {
        tft.drawCircle(128 - (tiltX + 64), -tiltY + 64, 5, ST77XX_BLACK);
    } else {
        tft.drawCircle(128 - (tiltX + 64), -tiltY + 64, 3, ST77XX_BLACK);
    }
}

tft.drawLine(128 - (LLx + 64), -LLy + 64, 128 - (URx + 64), -URy + 64, ST77XX_BLACK);
tft.drawLine(128 - (LLx2 + 64), -LLy2 + 64, 128 - (URx2 + 64), -URy2 + 64, ST77XX_BLACK);
tft.drawLine(128 - (headingX1 + 64), -headingY1 + 64, 128 - (headingX2 + 64), -headingY2 + 64, ST77XX_BLACK);
tft.drawCircle(128 - (headingOriginX + 64), -headingOriginY + 64, 3, ST77XX_BLACK);


}

}

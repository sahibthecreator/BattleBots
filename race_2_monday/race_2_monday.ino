#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hcsr04.h>
#include <ESP32Servo.h>
#include <hcsr04.h>

#define TRIG_PIN 25
#define ECHO_PIN 26


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
HCSR04 left_dist_sensor(TRIG_PIN, ECHO_PIN, 20, 4000);

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to


long duration;
float distanceCm;

bool checked = false;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  ledcAttachPin(led_gpio, 0); // assign a led pins to a channel
  ledcAttachPin(led_gpio1, 1); // assign a led pins to a channel
  ledcAttachPin(back1, 2); // assign a led pins to a channel
  ledcAttachPin(back2, 3); // assign a led pins to a channel

  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(1, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(3, 4000, 8); // 12 kHz PWM, 8-bit resolution



  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  //------Distance Sensor Test----------

    if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
    }
}

// the loop routine runs over and over again forever:
void loop() {



  //LineTracking()

    
    if (getDistanceF() > 200) {
        goForward(180);
    } else {
        stopCar();
        if (getDistanceR() > getDistanceL()) {
        while (getDistanceF() < 200 && getDistanceL() > 10) { // rotation algorithm
            turnRight(190);
            delay(50);
        }
        } else {
        while (getDistanceF() < 200 && getDistanceR() > 10) { // rotation algorithm
            turnLeft(190);
            delay(50);
        }
        }
    }


  }


int goForward(int speed) {
  ledcWrite(0, speed);
  ledcWrite(1, speed);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

int goBack(int speed) {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, speed);
  ledcWrite(3, speed);
}

int turnLeft(int speed) {
  ledcWrite(0, speed);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, speed);
}

int turnRight(int speed) {
  ledcWrite(0, 0);
  ledcWrite(1, speed);
  ledcWrite(2, speed);
  ledcWrite(3, 0);
}

int stopCar() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

int lookLeft() {
  ledcWrite(6, 7864);
  delay(500);
}

int lookRight() {
  ledcWrite(6, 1638);
  delay(500);
}

int lookForward() {
  int value = map(90, 0, 180, 1638, 7864);
  ledcWrite(6, value);
  delay(500);
}

int getDistanceF() {
  return middle_dist_sensor.distanceInMillimeters();
}

int getDistanceR() {
  return right_dist_sensor.distanceInMillimeters();
}

int getDistanceL() {
  return left_dist_sensor.distanceInMillimeters();
}

int findWay() {
  VL53L0X_RangingMeasurementData_t measure;
  int distance = 0;
  int distance1 = 0;

  stopCar();
  delay(500);

  for (int i = 0; i < 20; i++) {
    turnRight(150);
    lox.rangingTest(&measure, false);
    if (distance < measure.RangeMilliMeter) {
      distance = measure.RangeMilliMeter;
    }
    delay(6);
  }


  stopCar();
 
  delay(500);
  for (int i = 0; i < 20; i++) {
    turnLeft(175);
    lox.rangingTest(&measure, false);
    if (distance1 < measure.RangeMilliMeter) {
      distance1 = measure.RangeMilliMeter;
    }
    delay(12);
  }

  stopCar();
  delay(500);
 
  if (distance < distance1 )
  {
    return 1;
  } else {

    return 0;
  }

}


int displayDistance() {
  VL53L0X_RangingMeasurementData_t measure;


  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println(measure.RangeMilliMeter);
  display.display();
}

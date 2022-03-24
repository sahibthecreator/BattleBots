#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hcsr04.h>
#include <ESP32Servo.h>
#include <hcsr04.h>

#define TRIG_PIN 14
#define ECHO_PIN 12

#define TRIG_PIN_R 2
#define ECHO_PIN_R 15

#define TRIG_PIN_L 19
#define ECHO_PIN_L 21

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define SOUND_SPEED 0.034

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
HCSR04 middle_dist_sensor(TRIG_PIN, ECHO_PIN, 20, 4000);

HCSR04 right_dist_sensor(TRIG_PIN_R, ECHO_PIN_R, 20, 4000);

HCSR04 left_dist_sensor(TRIG_PIN_L, ECHO_PIN_L, 20, 4000);

Servo myservo;  // create servo object to control a servo

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to

int leftSensor = 39;
int rightSensor = 34;
int middleSensor = 13;


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




  //-----------Line Tracking Sensor----------
  pinMode (leftSensor, INPUT); // sensor pin INPUT
  pinMode (rightSensor, INPUT); // sensor pin INPUT

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  //------Distance Sensor Test----------
  /*
    if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
    }
  */
}

// the loop routine runs over and over again forever:
void loop() {
  //VL53L0X_RangingMeasurementData_t measure;
  //lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!


  //LineTracking();

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
  //if (getDistanceF() > 200 && getDistance)


    /*
      if (getDistanceF() > 20) {
        goForward(200);
        Serial.print("Distance (cm): ");
        Serial.println(getDistance());
        checked = false;
      }
      else {
        stopCar();
        if (checked == false) {
        checked = true;

          lookRight();
          int right = getDistance();
          Serial.print("Right Distance (cm): ");
          Serial.println(getDistance());
          //delay(5000);
          lookLeft();
          int left = getDistance();
          Serial.print("Left Distance (cm): ");
          Serial.println(getDistance());
          // delay(5000);
          //Serial.println("left sensor: ");
          //Serial.println(hcsr04.distanceInMillimeters());
          if (right > left) {
            Serial.println("Right MORE: ");
            lookForward();
            int distance = getDistance();

            while (distance < 20) { // rotation algorithm
              turnRight(170);

              Serial.print("Turning Right || Distance (cm): ");
              Serial.println(getDistance());
              distance = getDistance();
              delay(100);
            }
          } else {
            Serial.println("LEFT MORE: ");
            lookForward();
            int distance = getDistance();

            while (distance < 20) { // rotation algorithm
              turnLeft(170);

              Serial.print("Turning Left || Distance (cm): ");
              Serial.println(getDistance());
              distance = getDistance();
              delay(100);
            }
          }

        }
      }
    */


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
  //turnRight(175);
  //delay(600);

  stopCar();
  //lox.rangingTest(&measure, false);
  //Serial.println("on right distance is ");
  //Serial.println(measure.RangeMilliMeter);
  //distance = measure.RangeMilliMeter;
  delay(500);
  for (int i = 0; i < 20; i++) {
    turnLeft(175);
    lox.rangingTest(&measure, false);
    if (distance1 < measure.RangeMilliMeter) {
      distance1 = measure.RangeMilliMeter;
    }
    delay(12);
  }
  //turnLeft(175);
  //delay(1200);
  stopCar();
  delay(500);
  //lox.rangingTest(&measure, false);
  //Serial.println("on left distance is ");
  //Serial.println(measure.RangeMilliMeter);
  if (distance < distance1 )
  {
    return 1;
  } else {
    /*turnRight(175);
      delay(920);
      goForward(0); // stop
      break; */
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

//---------------------------------------------
//------------LINE TRACKING FUNCTION-----------
//---------------------------------------------
int LineTracking() {
  int sensorL = analogRead (leftSensor);
  int sensorR = analogRead (rightSensor);
  int sensorM = analogRead (middleSensor);
  int vSpeed =  240;        // MAX 255  220-240 speed best
  int turn_speed = 180;    // MAX 255 180 speed best
  int turn_delay = 0;

  Serial.print(F("Right sensor = "));
  Serial.println((sensorR));
  Serial.print(F("Middle sensor = "));
  Serial.println((sensorM));
  Serial.print(F("Left sensor = "));
  Serial.println((sensorL));
  if (sensorR > 150 && sensorL < 150)
  {
    Serial.println("turning right");
    while (sensorR > 150 && sensorL < 150 && sensorM < 150) {
      turnRight(turn_speed);
      sensorL = analogRead (leftSensor);
      sensorR = analogRead (rightSensor);
      sensorM = analogRead (middleSensor);
      //lookLeft();
    }

    //delay(turn_delay);

  }
  if (sensorR < 150 && sensorL > 150)
  {
    Serial.println("turning left");
    while (sensorR < 150 && sensorL > 150 && sensorM < 150) {
      turnLeft(turn_speed);
      sensorL = analogRead (leftSensor);
      sensorR = analogRead (rightSensor);
      sensorM = analogRead (middleSensor);
      //lookRight();
    }


    //delay(turn_delay);
  }

  if (sensorR < 150 && sensorL < 150 && sensorM > 150)
  {
    Serial.println("going forward");

    goBack(vSpeed);

  }

  if (sensorR > 150 && sensorL > 150)
  {
    Serial.println("stop");

    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);

  }

}

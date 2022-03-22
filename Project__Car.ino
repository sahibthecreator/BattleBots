#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hcsr04.h>
#include <ESP32Servo.h>

#define TRIG_PIN 14
#define ECHO_PIN 12

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);
Servo myservo;  // create servo object to control a servo

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to

int leftSensor = 39;
int rightSensor = 34;
int middleSensor = 13;

int servoPin = 19;
int pos = 0;

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

  //------------Servo Motor--------------
  ledcSetup(6, 50, 16); // channel 2, 50 Hz, 16-bit width
  ledcAttachPin(19, 6);   // GPIO 19 assigned to channel 2


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




  LineTracking();
/*
  Serial.println(hcsr04.distanceInMillimeters());
  if (hcsr04.distanceInMillimeters() > 300) {
    goForward(220);
  }
  else {
    stopCar();
    lookRight();
    int right = hcsr04.distanceInMillimeters();
    //Serial.println("right sensor: ");
    //Serial.println(hcsr04.distanceInMillimeters());
    lookLeft();
    int left = hcsr04.distanceInMillimeters();
    //Serial.println("left sensor: ");
    //Serial.println(hcsr04.distanceInMillimeters());
    if (right > left) {
      //Serial.println("right MORE: ");
      lookForward();
      while (hcsr04.distanceInMillimeters() < 300) {
        turnRight(200);
        //lox.rangingTest(&measure, false);
      }
    } else {
      //Serial.println("LEFT MORE: ");
      lookForward();
      while (hcsr04.distanceInMillimeters() < 300) {
        turnLeft(200);
        //lox.rangingTest(&measure, false);
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

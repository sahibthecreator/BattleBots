 #include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hcsr04.h>
#include <ESP32Servo.h>

#define TRIG_PIN 14
#define ECHO_PIN 12

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
@ -11,20 +16,25 @@
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);


// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

 void setup() {
  ledcSetup(2, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(3, 4000, 8); // 12 kHz PWM, 8-bit resolution

  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  //------Distance Sensor Test----------
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

// the loop routine runs over and over again forever:
void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!


  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  //displayDistance();

  }

}


int goForward(int speed) {
  ledcWrite(0, speed);
  ledcWrite(1, speed);
@ -108,6 +142,83 @@ int turnRight(int speed){
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

}
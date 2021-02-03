/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#define PHOBOS_USB_TRIGGER_1 2
#define PHOBOS_USB_TRIGGER_2 2

#define PHOBOS_GIGE_TRIGGER_1 3
#define PHOBOS_GIGE_TRIGGER_2 2

#define TITANIA_USB_TRIGGER_1 4
#define TITANIA_USB_TRIGGER_2 6

//Choose trigger pins based on the camera being used
#define CAMERA_TRIGGER_1 TITANIA_USB_TRIGGER_1 
#define CAMERA_TRIGGER_2 TITANIA_USB_TRIGGER_2

//Comment out when not using the IMU
#define USE_IMU
#ifdef USE_IMU
  #include <SoftwareSerial.h>
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
  
  #define TITANIA_USB_SS_RX 2
  #define TITANIA_USB_SS_TX 3

  //Choose software serial pins based on the camera being used
  #define CAMERA_SS_RX TITANIA_USB_SS_RX 
  #define CAMERA_SS_TX TITANIA_USB_SS_TX

  //Setup software serial
  SoftwareSerial softSerial(CAMERA_SS_RX, CAMERA_SS_TX);

  //Setup imu device
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
#endif

double frame_delay;      // amount of time between triggered (1/fps)
int trigger_time = 10;   // time for trigger to be registered by camera
double fps = 5;          // inital fps
String inString = "";    // string to hold input

void setup() {
  Serial.begin(115200);
  #ifdef USE_IMU
    softSerial.begin(9600);
    /* Initialise the IMU sensor */
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    } else {
      bno.setExtCrystalUse(true);
    }
  #endif
  pinMode(CAMERA_TRIGGER_1, OUTPUT);
  pinMode(CAMERA_TRIGGER_2, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  frame_delay = 1000/fps;
  
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(CAMERA_TRIGGER_1, HIGH);
  digitalWrite(CAMERA_TRIGGER_2, HIGH);
  delay(trigger_time);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(CAMERA_TRIGGER_1, LOW);
  digitalWrite(CAMERA_TRIGGER_2, LOW);
  delay(frame_delay-trigger_time);

  // Read trigger frame rate from serial
  if (Serial.available() > 0) { //Only read if data is available
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      fps = inString.toInt();
      // clear the string for new input:
      inString = "";
    }
  }

  #ifdef USE_IMU
    // Get a new IMU sensor event
    sensors_event_t event; 
    bno.getEvent(&event);
    
    // Display the floating point data
    softSerial.print(event.orientation.x, 4);
    softSerial.print(",");
    softSerial.print(event.orientation.y, 4);
    softSerial.print(",");
    softSerial.print(event.orientation.z, 4);
    softSerial.println();
  #endif
}

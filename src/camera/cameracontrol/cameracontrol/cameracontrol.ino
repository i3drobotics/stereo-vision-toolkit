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
#define CAMERA_TRIGGER_1 PHOBOS_USB_TRIGGER_1 
#define CAMERA_TRIGGER_2 PHOBOS_USB_TRIGGER_2

double frame_delay;      // amount of time between triggered (1/fps)
int trigger_time = 10;   // time for trigger to be registered by camera
double fps = 5;          // inital fps
String inString = "";    // string to hold input

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
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

  while (Serial.available() > 0) {
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
}

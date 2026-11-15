// Pan/tilt servo controller.
// Protocol: one ASCII line "pan,tilt\n" at 115200 baud, both angles in degrees (0-180).
#include <Servo.h>

const int PAN_PIN = 9;
const int TILT_PIN = 10;
const int HOME_PAN = 90;
const int HOME_TILT = 30;

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);  // default is 1000 ms, which would stall the loop on a partial line
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(HOME_PAN);
  tiltServo.write(HOME_TILT);
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    int pan, tilt;
    // Only move on a well-formed line; garbage must not slam the servos to 0.
    if (sscanf(line.c_str(), "%d,%d", &pan, &tilt) == 2) {
      panServo.write(constrain(pan, 0, 180));
      tiltServo.write(constrain(tilt, 0, 180));
    }
  }
}

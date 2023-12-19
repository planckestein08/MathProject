#include <Servo.h>

// Define the servo and joystick pins
Servo myServo;
const int joyPinX = A0;       // Connect joystick X-axis to A0
const int joyPinY = A1;       // Connect joystick Y-axis to A1
const int servoPin = 9;       // Connect servo signal to pin 9
const int laserPin = 10;      // Connect laser module to pin 10
const int pushButtonPin = 7;  // Connect push-button module to pin 7

bool toggleState = false;     // Variable to store toggle state
bool buttonPressed = false;   // Variable to track button press
int lockedAngle = 90;         // Default locked angle
int currentAngle = 90;        // Current angle of the servo
int fineTuneValue = 0;        // Fine-tune adjustment value

void setup() {
  myServo.attach(servoPin);
  pinMode(laserPin, OUTPUT);
  pinMode(pushButtonPin, INPUT_PULLUP); // Assuming the button is connected to pin 7 and has a pull-up resistor
  Serial.begin(9600);
}

void loop() {
  // Read joystick positions
  int joyValueX = analogRead(joyPinX);
  int joyValueY = analogRead(joyPinY);

  // Check for button press
  if (digitalRead(pushButtonPin) == LOW && !buttonPressed) {
    toggleState = !toggleState;  // Toggle the state when the button is pressed
    buttonPressed = true;        // Set button press flag

    if (toggleState) {
      lockedAngle = currentAngle; // Save the current angle when locking
    }

    delay(50);  // Debounce delay
  }

  // Reset button press flag when button is released
  if (digitalRead(pushButtonPin) == HIGH) {
    buttonPressed = false;
  }

  // Fine-tune adjustment based on joystick Y-axis
  fineTuneValue = map(joyValueY, 0, 1023, -10, 10);

  // If the servo is locked, set the angle to the locked angle with fine-tuning
  if (toggleState) {
    myServo.write(lockedAngle + fineTuneValue);
    digitalWrite(laserPin, HIGH); // Turn on the laser module when locked
  } else {
    // Map the joystick values (0-1023) to servo angle (30-150) with fine-tuning
    int servoAngle = map(joyValueX, 502, 1023, 30, 140) + fineTuneValue;

    // Move the servo to the mapped angle
    myServo.write(servoAngle);
    currentAngle = servoAngle; // Update current angle

    digitalWrite(laserPin, joyValueX > 800 ? HIGH : LOW); // Turn on the laser module when joystick position is above the threshold
  }

  // Print values for debugging
  Serial.print("Joystick X: ");
  Serial.print(joyValueX);
  Serial.print(" | Joystick Y: ");
  Serial.print(joyValueY);
  Serial.print(" | Servo Angle: ");
  Serial.print(myServo.read());  // Print the current servo angle
  Serial.print(" | Laser: ");
  Serial.print(digitalRead(laserPin) == HIGH ? "ON" : "OFF");
  Serial.print(" | Toggle State: ");
  Serial.println(toggleState ? "Locked" : "Unlocked");

  delay(15); // Adjust the delay as needed
}

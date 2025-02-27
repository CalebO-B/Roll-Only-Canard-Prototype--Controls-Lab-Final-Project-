#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "SparkFun_ProDriver_TC78H670FTG_Arduino_Library.h"
#include "Arduino_LED_Matrix.h"

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);

sh2_SensorValue_t sensorValue;

// Stepper declaration
PRODRIVER Stepper;

// Variable intiation
int Gravity = 1; // Make sure stepper doesn't move on startup
int Step = 0; // Tracks steps from a vertical canard position
bool Direction = 0; // Direction stepper moves
int Steplimit = 22; // Max step limit for canard control, out of 34 

// LED Matrix definitions
ArduinoLEDMatrix matrix;
// On Frame
unsigned long on[] = {
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF
};

// Off Frame
unsigned long off[] = {
  0x0000000,
  0x0000000,
  0x0000000
};

void setup() {
  Serial.begin(115200);

  // IMU startup
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Wire.setClock(400000);
  Serial.println("BNO08x Found!");

  // Set what reports come from the IMU
  setReports();

  // Motor Setup
  Stepper.begin();

  // LED matrix Setup
  matrix.begin();
  matrix.loadFrame(off);
}

// IMU Sensor events to poll for, commented if statements are for other events I didn't end up needing
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) { // 10000 is the time in milliseconds for this event to send a new reading
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_GRAVITY, 20000)) { // 20000 is the time in milliseconds for this event to send a new reading, It's double the gyro because it was overcrowding the report buffer and the gryo couldn't be read
    Serial.println("Could not enable gravity vector");
  }
}

void loop() {
  delay(10);
  // Check if the sensor was reset and reset the reports
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  // Check if the sensor reported an event
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
    // Read what event was triggered and then call the corresponding block of code
    switch (sensorValue.sensorId) {

      case SH2_GYROSCOPE_CALIBRATED: 
        // Prints out relevant variables
        Serial.println("Gryo");
        Serial.print("Gravity: "); Serial.println(Gravity);
        Serial.print("Step: "); Serial.println(Step);
        Serial.print("Gyro - x: ");
        Serial.println(sensorValue.un.gyroscope.x);
        // Checks if the rocket is not vertical and if the gyro has a large enoug reading to warrant a canard movement
        if ((Gravity == 0) && (sensorValue.un.gyroscope.x > 1)) { // Change gyro axis and direction of rotation depending on final orientation of the IMU
          Direction = 0;
          int Step2move2 = map(sensorValue.un.gyroscope.x, 1, 20, 1, Steplimit); // Maps the output of the gyro to the number of steps within a range
          // The steps are mapped to a positive and negative limit from a vertical canard position, which is where the next if statements determine the direction of how the motor should turn
          Serial.print("Step 2 move 2: "); Serial.print(Step2move2); Serial.println();
          if (Step2move2 > Step) { // If the step we need to go to 
            Stepper.step(abs(Step2move2-Step), Direction, 1);
            Step = Step2move2; // Sets the Step to where the canard is after the movement
          }
          else if (Step2move2 < Step) {
            Stepper.step(abs(Step2move2-Step), !Direction, 1);
            Step = Step2move2;
          }
          else if (Step2move2 == Step) {}
        }

        else if ((Gravity == 0) && (sensorValue.un.gyroscope.x < -1)) {
          Direction = 1;
          int Step2move2 = map(sensorValue.un.gyroscope.x, -1, -20, -1, -Steplimit);
          Serial.print("Step 2 move 2: "); Serial.print(Step2move2); Serial.println();
          if (Step2move2 < Step) {
            Stepper.step(abs(Step2move2-Step), Direction, 1);
            Step = Step2move2;
          }
          else if (Step2move2 > Step) {
            Stepper.step(abs(Step2move2-Step), !Direction, 1);
            Step = Step2move2;
          }
          else if (Step2move2 == Step) {}
        }
        break;
      case SH2_GRAVITY: 
        Serial.println("Gravity");
        Serial.print("Gravity: "); Serial.println(Gravity);
        Serial.print("Gravity - z: ");
        Serial.println(sensorValue.un.gravity.z);

        // Checks if the gravity vector is within range for it to be considered upright and the gravity variable is not triggered
        if (((sensorValue.un.gravity.z > 9) || (sensorValue.un.gravity.z < -9)) && (Gravity == 0)) { // Change gravity axis depending on final orientation of the IMU
          // Sets the gravity variable to be triggered
          Gravity = 1;
          Serial.print("Step G->1: "); Serial.println(Step);
          // Steps the required amount to make the canards lay flat
          Stepper.step(abs(34-Step), Direction, 2);
          // Turns on the LED matrix
          matrix.loadFrame(on);
          // Sets the current step position to be flat
          Step = 34;
        }
        // Checks if we come out of vertical and the gravit conditions is not turned off
        else if (((sensorValue.un.gravity.z < 9) && (sensorValue.un.gravity.z > -9)) && (Gravity == 1)) {
          // Sets the gravity condition to be off and our step position to be vertical
          Gravity = 0;
          Step = 0;
          Serial.print("Step G->0: "); Serial.println(Step);
          // Moves the required amount to be in a verical position and turns the LED matrix off
          Stepper.step(34, !Direction, 2);
          matrix.loadFrame(off);
        }
        break;
    }

  
}


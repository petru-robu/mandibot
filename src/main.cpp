#include <Arduino.h>
#include <Servo.h>

// --- User Constants ---
const int TEMP_STG = 11;
const int TEMP_DRP = 5;
const int MAS_STG = 10;
const int MAS_DRP = 6;
const int LATPTER_STG = 9;
const int LATPTER_DRP = 2;
const int MEDPTER_STG = 8;
const int MEDPTER_DRP = 3;

const int closed_TEMP_STG = 50;

const int open_TEMP_STG = 0;
const int closed_TEMP_DRP = 50;
const int open_TEMP_DRP = 0;

const int closed_MAS_STG = 0;
const int open_MAS_STG = 50;
const int closed_MAS_DRP = 50;
const int open_MAS_DRP = 0;
const int closed_LATPTER_STG = 0;
const int open_LATPTER_STG = 100;
const int closed_LATPTER_DRP = 100;
const int open_LATPTER_DRP = 0;
const int closed_MEDPTER_STG = 0;
const int open_MEDPTER_STG = 50;
const int closed_MEDPTER_DRP = 50;
const int open_MEDPTER_DRP = 0;

const int NUM_MOTORS = 8;

struct Motor {
    int pin;
    Servo servo;
    int currentAngle;
    int openLim;
    int closedLim;
};

// Map hardware to array
Motor motors[NUM_MOTORS] = {
  {TEMP_STG, Servo(), 0, open_TEMP_STG, closed_TEMP_STG},
  {TEMP_DRP, Servo(), 0, open_TEMP_DRP, closed_TEMP_DRP},
  {MAS_STG, Servo(), 0, open_MAS_STG, closed_MAS_STG},
  {MAS_DRP, Servo(), 0, open_MAS_DRP, closed_MAS_DRP},
  {LATPTER_STG, Servo(), 0, open_LATPTER_STG, closed_LATPTER_STG},
  {LATPTER_DRP, Servo(), 0, open_LATPTER_DRP, closed_LATPTER_DRP},
  {MEDPTER_STG, Servo(), 0, open_MEDPTER_STG, closed_MEDPTER_STG},
  {MEDPTER_DRP, Servo(), 0, open_MEDPTER_DRP, closed_MEDPTER_DRP}
};

void setup() {
    Serial.begin(115200); // Using faster baud rate for real-time sliders
    Serial.setTimeout(10); // Fast timeout for responsive reading

    // Attach servos and set to 100% (Closed) by default
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].servo.attach(motors[i].pin);
        motors[i].servo.write(motors[i].closedLim);
        motors[i].currentAngle = motors[i].closedLim;
    }
    Serial.println("Arduino ready. Listening for slider data...");
}

void loop() {
    // Check if computer sent any data
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        int separatorIndex = data.indexOf(':');

        // Validate format "Index:Percentage"
        if (separatorIndex != -1) {
            int motorIndex = data.substring(0, separatorIndex).toInt();
            float percentage = data.substring(separatorIndex + 1).toFloat();

            if (motorIndex >= 0 && motorIndex < NUM_MOTORS) {
                // Map 0-100% to the specific open/closed angles defined for this motor
                float safePercent = constrain(percentage, 0.0, 100.0);
                float openAngle = motors[motorIndex].openLim;
                float closedAngle = motors[motorIndex].closedLim;

                int targetAngle = openAngle + ((safePercent / 100.0) * (closedAngle - openAngle));

                motors[motorIndex].servo.write(targetAngle);
                motors[motorIndex].currentAngle = targetAngle;
            }
        }
    }
}
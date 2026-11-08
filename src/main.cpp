#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

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

// --- Audio (DY-SV8F / DY-SV5W MP3 module reading the SD card) ---
// The hardware UART is taken by the PC link, so the module is driven over SoftwareSerial at 9600 baud.
const int AUDIO_TX = 4;  // Arduino TX -> module RX
const int AUDIO_RX = 7;  // not used by the module commands, but SoftwareSerial needs a pin
SoftwareSerial audio(AUDIO_RX, AUDIO_TX);

// Frame: header, command, length, checksum
const byte AUDIO_PLAY[] = {0xAA, 0x02, 0x00, 0xAC};  // play the current/first track
const byte AUDIO_STOP[] = {0xAA, 0x04, 0x00, 0xAE};  // stop playback

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

    audio.begin(9600);

    // Attach servos and set to 50% (halfway between open and closed) by default
    for (int i = 0; i < NUM_MOTORS; i++) {
        int startAngle = motors[i].openLim + (motors[i].closedLim - motors[i].openLim) / 2;
        motors[i].servo.attach(motors[i].pin);
        motors[i].servo.write(startAngle);
        motors[i].currentAngle = startAngle;
    }
    Serial.println("Arduino ready. Listening for slider data...");
}

void loop() {
    // Check if computer sent any data
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        data.trim();

        // Audio commands from the PC: "PLAY" / "STOP"
        if (data == "PLAY") {
            audio.write(AUDIO_PLAY, sizeof(AUDIO_PLAY));
            return;
        }
        if (data == "STOP") {
            audio.write(AUDIO_STOP, sizeof(AUDIO_STOP));
            return;
        }

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
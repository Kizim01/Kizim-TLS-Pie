/*
 * Title       LidarHDMicroview
 * by          Donny Mott
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Description:
 *   Single axis stepper motor control with OLED dsiplay using a Microview controller. This software is for use with a DIY Pie Powered Terrestrial Laser Scanner
 *   Button inputs to select various scan settings and to stop and reset
 *   180 degree scan at 1 deg/sec
 *   360 degree scan at 2 deg/sec
 *   360 degree scan at 1 deg/sec
 *
 *   When scan buttons are pressed an output is fired to tell a Raspberry Pie to start recording the Lidar data in a PCAP file.
 *   The code below is designed to work with a 0.9deg 400 step per rev stepper motor and a 50:1 gear reducer. The code is currenlty set to 32 microsteps.
 *   The code is easily changed to support whatever stepper motor and gearbox you choose to use.
 *
 * Author: Donny Mott
 *
 *   3dmapmaker@gmail.com
 *
 *
 */
#include <stdio.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SpeedyStepper.h>
#include <MicroView.h>

const int pan360s = A0; //button
const int pan360f = A1; //button
const int pan180s = A2; //button
const int KILL = 2; //button
const int MOTOR_EN_PIN = 6; //pan motor enable
const int MOTOR_STEP_PIN = 5; //pan motor step
const int MOTOR_DIRECTION_PIN = 3; //pan motor direction
const int RECORDSTART = 7; // output to start bag record on Pie (avoid A4/A5, which MicroView uses for the OLED)
const int RECORDSTOP = 8; // output to stop bag record on Pie
const int PISTATUS = 4; // input from Pi status pulse line (through level shifter)

bool buttonS360pressed = false;
bool buttonF360pressed = false;
bool buttonS180pressed = false;

// Set by the KILL interrupt only. The ISR does nothing else: it must not touch
// the OLED and must not call delay(), because delay() relies on the Timer0
// interrupt, which is masked while an ISR runs. The old code did both and hung
// the board permanently on the first kill press.
volatile bool killRequested = false;

// Any latched fault (kill pressed, Pi abort, Pi ack timeout). While this is set
// the main loop does nothing, so the fault message stays on screen until reset.
bool faultLatched = false;

bool piStatusLast = HIGH;
int piStatusPulseCount = 0;
int piAbortCode = 0;
bool readyScreenShown = false;
bool piRecordingAckReceived = false;
bool scanInProgress = false;
unsigned long piStatusLastEdgeMs = 0;
unsigned long piStatusLastPulseMs = 0;

const unsigned long PI_STATUS_MIN_PULSE_MS = 20;
const unsigned long PI_STATUS_CODE_GAP_MS = 500;
const unsigned long PI_ACK_TIMEOUT_MS = 4000;
const int PI_STATUS_CODE_RECORDING_ACK = 8;

const unsigned long BUTTON_DEBOUNCE_MS = 30;
const unsigned long KILL_DECEL_TIMEOUT_MS = 3000;

const int SCREEN_BUFFER_BYTES = 384; // 64 x 48 / 8

SpeedyStepper stepper;

// 'lidar logo, 64x48px -- held in flash, not RAM. drawBitmap() only reads plain
// RAM, so drawLogo() copies it into the screen buffer with pgm_read_byte().
const uint8_t logo [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x30, 0x18, 0x18, 0x0C, 0x0C, 0x0C, 0x06, 0x06, 0x86, 0x86,
0x86, 0x86, 0x06, 0x06, 0x0C, 0x0C, 0x0C, 0x18, 0x18, 0x30, 0x70, 0xE0, 0xC0, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0,
0xFC, 0x1E, 0x07, 0x01, 0x00, 0x00, 0xE0, 0x78, 0x1C, 0x0E, 0x06, 0x03, 0x83, 0xC3, 0xC1, 0xC1,
0xC1, 0xC1, 0xC3, 0x83, 0x03, 0x06, 0x0E, 0x1C, 0x78, 0xE0, 0x00, 0x00, 0x01, 0x07, 0x1E, 0xFC,
0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F,
0xFF, 0x80, 0x00, 0x00, 0x00, 0x07, 0x3F, 0x70, 0x40, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x3F,
0x3F, 0x3F, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x78, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x80, 0xFF,
0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x01, 0x07, 0x0E, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xF8, 0x78, 0x78, 0x78,
0x78, 0x78, 0x78, 0xF8, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0E, 0x07, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0xE3, 0x39, 0x0C, 0x06, 0x02, 0x02, 0x03,
0x03, 0x02, 0x02, 0x06, 0x0C, 0x39, 0xE3, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7C, 0x71, 0x67, 0x4C, 0x58, 0x10, 0x30, 0x30,
0x30, 0x30, 0x10, 0x58, 0x4C, 0x67, 0x71, 0x7C, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void drawLogo() {
    uint8_t *buffer = uView.getScreenBuffer();
    for (int i = 0; i < SCREEN_BUFFER_BYTES; i++) {
        buffer[i] = pgm_read_byte(&logo[i]);
    }
}

void showReadyScreen() {
    uView.clear(PAGE);
    uView.setFontType(1);
    uView.setCursor(10,6);
    uView.print("READY");
    uView.setFontType(0);
    uView.setCursor(8,26);
    uView.print("PRESS A0/A1/A2");
    uView.setCursor(14,38);
    uView.print("TO SCAN");
    uView.display();
    readyScreenShown = true;
}

void showPiAckWaitScreen() {
    uView.clear(PAGE);
    uView.setFontType(1);
    uView.setCursor(7,6);
    uView.print("WAIT PI");
    uView.setFontType(0);
    uView.setCursor(7,28);
    uView.print("ARMING REC...");
    uView.display();
}

void showPiAckTimeoutMessage() {
    uView.clear(PAGE);
    uView.setFontType(1);
    uView.setCursor(2,7);
    uView.print("PI TIMEOUT");
    uView.setFontType(0);
    uView.setCursor(2,24);
    uView.print("NO REC ACK");
    uView.setCursor(2,36);
    uView.print("PRESS RESET");
    uView.display();
    readyScreenShown = false;
}

void showRecLostMessage() {
    uView.clear(PAGE);
    uView.setFontType(1);
    uView.setCursor(4,7);
    uView.print("REC LOST");
    uView.setFontType(0);
    uView.setCursor(8,26);
    uView.print("PI ABORTED");
    uView.display();
}

void showStoppedMessage() {
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1);
    uView.setCursor(1,7);
    uView.print("STOPPED");
    uView.display();
    delay(500);
    uView.setFontType(0);
    uView.setCursor(18,24);
    uView.print("PRESS");
    uView.setCursor(18,36);
    uView.print("RESET");
    uView.display();
    readyScreenShown = false;
}

void drawRecBadge() {
    uView.setFontType(0);
    uView.setCursor(45,0);
    uView.print("REC");
}

void showPiAbortMessage(int code) {
    const char* reason = "UNKNOWN";
    if (code == 1) reason = "START TIMEOUT";
    else if (code == 2) reason = "NO INTERFACE";
    else if (code == 3) reason = "LIDAR OFFLINE";
    else if (code == 4) reason = "TCPDUMP ERROR";
    else if (code == 5) reason = "EMPTY PCAP";
    else if (code == 6) reason = "INTERRUPTED";
    else if (code == 7) reason = "TOOL MISSING";

    uView.clear(PAGE);
    uView.setFontType(0);
    uView.setCursor(2,4);
    uView.print("PI ABORTED");
    uView.setCursor(2,20);
    uView.print(reason);
    uView.setCursor(2,36);
    uView.print("CODE ");
    uView.print(code);
    uView.display();
    readyScreenShown = false;
}

// The whole of the KILL interrupt. Everything else happens in handleKill().
void killISR() {
    killRequested = true;
}

void handleKill() {
    killRequested = false;
    scanInProgress = false;

    // Decelerate under control rather than cutting the drive dead. Bounded so a
    // stalled driver cannot wedge us here.
    stepper.setupStop();
    unsigned long decelStart = millis();
    while (!stepper.processMovement() && (millis() - decelStart) < KILL_DECEL_TIMEOUT_MS) {
        ;
    }

    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable drive

    showStoppedMessage();
    faultLatched = true;
}

void checkPiStatus() {
    unsigned long now = millis();
    bool current = digitalRead(PISTATUS);

    if (current != piStatusLast) {
        unsigned long dt = now - piStatusLastEdgeMs;
        piStatusLastEdgeMs = now;

        // Count only LOW-going edges as pulses and ignore obvious bounce.
        if (current == LOW && dt >= PI_STATUS_MIN_PULSE_MS) {
            piStatusPulseCount++;
            piStatusLastPulseMs = now;
        }
        piStatusLast = current;
    }

    if (!faultLatched && piStatusPulseCount > 0 && (now - piStatusLastPulseMs) > PI_STATUS_CODE_GAP_MS) {
        int code = piStatusPulseCount;
        piStatusPulseCount = 0;

        if (code == PI_STATUS_CODE_RECORDING_ACK) {
            piRecordingAckReceived = true;
            return;
        }

        piAbortCode = code;
        faultLatched = true;
        if (scanInProgress == true) {
            showRecLostMessage();
            delay(700);
        }
        showPiAbortMessage(piAbortCode);
    }
}

// Runs a move to completion while still servicing the KILL button and the Pi
// status line. The old code used the blocking moveToPositionInRevolutions(),
// which meant neither was looked at for the 3-6 minutes the scan took.
// Returns false if the move was cut short, in which case the drive is already
// disabled and a fault screen is already up.
bool moveToRevolutionsWatched(float targetRevolutions) {
    stepper.setupMoveInRevolutions(targetRevolutions);

    while (!stepper.processMovement()) {
        if (killRequested == true) {
            handleKill();
            return false;
        }

        checkPiStatus();
        if (faultLatched == true) {
            stepper.setupStop();
            unsigned long decelStart = millis();
            while (!stepper.processMovement() && (millis() - decelStart) < KILL_DECEL_TIMEOUT_MS) {
                ;
            }
            digitalWrite(MOTOR_EN_PIN, HIGH); // disable drive
            return false;
        }
    }
    return true;
}

bool sendRecordStartAndWaitForPiAck() {
    piRecordingAckReceived = false;
    showPiAckWaitScreen();

    digitalWrite(RECORDSTART, LOW); // starts bag record
    delay(1);
    digitalWrite(RECORDSTART, HIGH);

    unsigned long startMs = millis();
    while ((millis() - startMs) < PI_ACK_TIMEOUT_MS) {
        if (killRequested == true) {
            handleKill();
            return false;
        }
        checkPiStatus();
        if (faultLatched == true) {
            return false;
        }
        if (piRecordingAckReceived == true) {
            return true;
        }
        delay(10);
    }

    scanInProgress = false;
    showPiAckTimeoutMessage();
    faultLatched = true; // otherwise loop() redraws READY over the message at once
    return false;
}

void setup() {
    Serial.begin(9600);
    delay(500);  // needed to be able to flash a new sketch more easily

  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

  // All four buttons are wired active-low, so enable the internal pull-ups.
  // Left floating, KILL in particular would fire its interrupt on noise alone.
  pinMode(pan360s, INPUT_PULLUP); //Start Motor @ 1 degree per second for 360 degrees
  pinMode(pan360f, INPUT_PULLUP); //Start Motor @ 2 degree per second for 360 degrees
  pinMode(pan180s, INPUT_PULLUP); //Start Motor @ 1 degree per second for 180 degrees
  pinMode(KILL, INPUT_PULLUP); //Stop Motor
  pinMode(MOTOR_EN_PIN, OUTPUT);
  pinMode(RECORDSTART, OUTPUT);
  pinMode(RECORDSTOP, OUTPUT);
    pinMode(PISTATUS, INPUT_PULLUP);

  // FALLING, not RISING: with a pull-up the pin idles high and goes low on
  // press, so RISING used to fire on release.
  attachInterrupt(digitalPinToInterrupt(KILL), killISR, FALLING);

  digitalWrite(MOTOR_EN_PIN, HIGH); // disable motor driver
  digitalWrite(RECORDSTART, HIGH); // makes sure not to trigger pie
  digitalWrite(RECORDSTOP, HIGH); // makes sure not to trigger pie

    uView.begin();              // start MicroView
    uView.clear(PAGE);          // erase the memory buffer, when next uView.display() is called, the OLED will be cleared.
    uView.display();            // display the content in the buffer memory


  // Display start
    uView.clear(PAGE);
    drawLogo();
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.setFontType(0); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(6,5);
    uView.print("STARTING"); // display string
    uView.display();
    delay(500);
    uView.setCursor(14,20);
    uView.print("LIDAR");
    uView.display();
    delay(500);
    uView.setCursor(10,36);
    uView.print("SCANNER");
    uView.display();

    delay(1500);
    uView.clear(PAGE);
    uView.display();

    // Ignore anything the KILL line did while the pins were still settling.
    killRequested = false;

    showReadyScreen();
}

void loop() {
    if (killRequested == true) {
        handleKill();
    }
    if (faultLatched == true) {
        delay(25);
        return;
    }
    checkPiStatus();
    if (faultLatched == true) {
        delay(25);
        return;
    }
    if (readyScreenShown == false) {
        showReadyScreen();
    }
  readButtons();
  actOnButtons();
}

// Confirms the level is still low after the bounce window before calling it a press.
bool buttonPressed(int pin) {
    if (digitalRead(pin) != LOW) {
        return false;
    }
    delay(BUTTON_DEBOUNCE_MS);
    return (digitalRead(pin) == LOW);
}

void readButtons() {
    buttonS360pressed = false;
    buttonF360pressed = false;
    buttonS180pressed = false;

    if (buttonPressed(pan360s)) {
        buttonS360pressed = true;
    }
    if (buttonPressed(pan360f)) {
        buttonF360pressed = true;
    }
    if (buttonPressed(pan180s)) {
        buttonS180pressed = true;
    }
}

void actOnButtons() {
    if (buttonS360pressed == true) {
        PANO360S();
    }
    if (buttonF360pressed == true) {
        PANO360F();
    }
    if (buttonS180pressed == true) {
        PANO180S();
    }
}


void PANO360S() {
    readyScreenShown = false;
    scanInProgress = true;
    digitalWrite(MOTOR_EN_PIN, LOW); // enable drive
    stepper.setStepsPerRevolution(640000); //CSF-14-50 harmonic drive 50:1, 32 microsteps, 0.9 deg motor
    stepper.setSpeedInRevolutionsPerSecond(0.0027778); // 1 degree per second, 360 degrees in 6 minutes
    stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
    stepper.setCurrentPositionInSteps(0);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("1 DEG/SEC");
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(7,4);
    uView.print("SCANNING");
    uView.setFontType(1);
    uView.setCursor(17,18);
    uView.print("360");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("6 Minutes");
    uView.display();
    if (!sendRecordStartAndWaitForPiAck()) {
        digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
        scanInProgress = false;
        return;
    }
    drawRecBadge();
    uView.display();

    // rotates 378 degrees. Add some rotation to be sure and record a full 360 after TCPDUMP starts
    if (!moveToRevolutionsWatched(1.05)) {
        return;
    }
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    delay(1000);
    stepper.setSpeedInRevolutionsPerSecond(0.1);
    if (!moveToRevolutionsWatched(1)) { // back to start
        return;
    }
    delay(100);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(12,7);
    uView.print("DONE"); // display string
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    showReadyScreen();
    scanInProgress = false;
    }

void PANO360F() {
    readyScreenShown = false;
    scanInProgress = true;
    digitalWrite(MOTOR_EN_PIN, LOW); // enable drive
    stepper.setStepsPerRevolution(640000);
    stepper.setSpeedInRevolutionsPerSecond(0.00556); // 2 degree per second, 360 degrees in 3 minutes
    stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
    stepper.setCurrentPositionInSteps(0);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("2 DEG/SEC");
    uView.display();
    delay(1000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(7,4);
    uView.print("SCANNING");
    uView.setFontType(1);
    uView.setCursor(17,18);
    uView.print("360");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("3 Minutes");
    uView.display();
    if (!sendRecordStartAndWaitForPiAck()) {
        digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
        scanInProgress = false;
        return;
    }
    drawRecBadge();
    uView.display();

    if (!moveToRevolutionsWatched(1.05)) { // rotates 378 degrees
        return;
    }
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    delay(1000);
    stepper.setSpeedInRevolutionsPerSecond(0.1);
    if (!moveToRevolutionsWatched(1)) { // back to start
        return;
    }
    delay(100);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(12,7);
    uView.print("DONE"); // display string
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    showReadyScreen();
    scanInProgress = false;
    }

void PANO180S() {
    readyScreenShown = false;
    scanInProgress = true;
    digitalWrite(MOTOR_EN_PIN, LOW); // enable drive
    stepper.setStepsPerRevolution(640000);
    stepper.setSpeedInRevolutionsPerSecond(0.0027778); // 1 degree per second, 360 degrees in 6 minutes
    stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
    stepper.setCurrentPositionInSteps(0);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("1 DEG/SEC");
    uView.display();
    delay(1000);
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.setFontType(0);
    uView.setCursor(7,4);
    uView.print("SCANNING");
    uView.setFontType(1);
    uView.setCursor(17,18);
    uView.print("180");
    uView.setFontType(0);
    uView.setCursor(5,36);
    uView.print("3 Minutes");
    uView.display();
    if (!sendRecordStartAndWaitForPiAck()) {
        digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
        scanInProgress = false;
        return;
    }
    drawRecBadge();
    uView.display();

    if (!moveToRevolutionsWatched(0.53)) { // rotates 190.8 degrees adds 10.8 degrees overlap
        return;
    }
    digitalWrite(RECORDSTOP, LOW); // stops bag record
    delay(10);
    digitalWrite(RECORDSTOP, HIGH);
    delay(2000);
    stepper.setSpeedInRevolutionsPerSecond(0.1);
    if (!moveToRevolutionsWatched(0)) { // back to start use this line when doing 180 degrees
        return;
    }
    delay(1000);
    digitalWrite(MOTOR_EN_PIN, HIGH); // disable driver
    uView.clear(PAGE);
    uView.display();
    delay(100);
    uView.clear(PAGE);
    uView.setFontType(1); // set font type 0: Numbers and letters. 10 characters per line (6 lines)
    uView.setCursor(12,7);
    uView.print("DONE"); // display string
    uView.display();
    delay(2000);
    uView.clear(PAGE);
    uView.display();
    showReadyScreen();
    scanInProgress = false;
    }

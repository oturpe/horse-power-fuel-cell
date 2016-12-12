#include <EEPROM.h>

// Select the appropriate config file
//#include "config-mean.h"
#include "config-transient.h"

// Analog input pin sensing the voltage
#define V_SENSE_PIN A4
// Analog input pin sensing the current
#define I_SENSE_PIN A3
// Analog input pin for ambient light sensor
#define AMBIENT_LIGHT_PIN A7
// Analog input pin for opacity sensor
#define OPACITY_PIN A2
// Analog input pin for temperature sensor
#define TEMPERATURE_PIN A6

// How many quatities are measured
#define QUANTITY_COUNT 5

// Pin for button that outputs saved readings
#define CONTROL_BUTTON_PIN A5

// Pwm output pin for solution pump
#define SOLUTION_PUMP_PIN 11
// Pin for recording indicator
#define RECORDING_INDICATOR_PIN A1

// Size of eeprom memory to read
#define EEPROM_SIZE 1024
// Marker to write for write start
#define WRITE_START_MARK 0xff
// Marker to write for write end
#define WRITE_END_MARK 0xfe

// How the control button must be pressed to register a button press
#define BUTTON_OUTPUT_DELAY 200
// How the control button must be pressed to start or stop writing values.
// Given in units of millisecond
#define BUTTON_WRITE_DELAY 2000

// How long the solution pump runs after it starts- Given in units of
// millisecond.
#define SOLUTION_PUMP_ON_TIME (10 * 1000)
// How long the solution pump stays off after it stops. Givnen in units of
// millisecond.
#define SOLUTION_PUMP_OFF_TIME (10 * 1000)
// Duty cycle of pwm used for the solution pump. Range is 0 .. 0xff for
// 0 % .. 100 %.
#define SOLUTION_PUMP_PWM_DUTY_CYCLE 170

bool isWriting = false;

// Eeprom address to write
int writeAddress = 0;

// If control button is pressed
bool wasButtonDown = false;

// The moment of time when button went down
unsigned long buttonDownTime = 0;
// The moment values were last written
unsigned long writeTime = 0;

// If solution pump is running
bool isSolutionPumpRunning = false;
// When solution pump was started or stopped the last time
unsigned long solutionPumpChangeTime = 0;

// How many blocks have been written
int blocksWritten = 0;

void setup() {
    analogReference(INTERNAL);

    pinMode(V_SENSE_PIN, INPUT);
    pinMode(I_SENSE_PIN, INPUT);
    pinMode(AMBIENT_LIGHT_PIN, INPUT);
    pinMode(OPACITY_PIN, INPUT);
    pinMode(TEMPERATURE_PIN, INPUT);
    pinMode(CONTROL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SOLUTION_PUMP_PIN, OUTPUT);
    pinMode(RECORDING_INDICATOR_PIN, OUTPUT);

    analogWrite(SOLUTION_PUMP_PIN, 0);
}

void loop() {
    unsigned long currentTime = millis();
    
    bool isButtonDown = digitalRead(CONTROL_BUTTON_PIN) == LOW; 

    if (isButtonDown && !wasButtonDown) {
        buttonDownTime = currentTime;
    }
    else if (!isButtonDown && wasButtonDown) {
        // Button was released
        int buttonDownLength = currentTime - buttonDownTime;

        if (buttonDownLength > BUTTON_WRITE_DELAY) {
            isWriting = !isWriting;
            if (isWriting) {
                 WriteStartMark();
            } else {
                WriteEndMark();
            }
        } else if (buttonDownLength > BUTTON_OUTPUT_DELAY) {
            outputReadings();
        }
    }

    wasButtonDown = isButtonDown;

    digitalWrite(RECORDING_INDICATOR_PIN, isWriting);
    if (isWriting && IsTimeToWrite(currentTime)) {
        WriteReadings();
        writeTime = currentTime;
    }

    NotifySolutionPumpTimer(currentTime);
}

void WriteStartMark() {
    if (writeAddress == EEPROM_SIZE - 1) {
        return;
    }

    EEPROM.write(writeAddress++, WRITE_START_MARK);
}

void WriteEndMark() {
    EEPROM.write(writeAddress++, WRITE_END_MARK);
    EEPROM.write(EEPROM_SIZE - 1, ++blocksWritten);
}

void WriteReadings() {
    // Last bit is amount of blocks
    if (writeAddress == EEPROM_SIZE - 1) {
        return;
    }

    int voltage = 0;
    int current = 0;
    int ambientLight = 0;
    int opacity = 0;
    int temperature = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int voltageSample = analogRead(V_SENSE_PIN);
        voltage += map(voltageSample, 0, 0x3ff, 0, 0xfd);

        int currentSample = analogRead(I_SENSE_PIN);
        current += constrain(currentSample, 0, 0xfd);

        int ambientLightSample = analogRead(AMBIENT_LIGHT_PIN);
        ambientLight += map(ambientLightSample, 0, 0x3ff, 0, 0xfd);

        int opacitySample = analogRead(OPACITY_PIN);
        opacity += map(opacitySample, 0, 0x3ff, 0, 0xfd);

        int temperatureSample = analogRead(TEMPERATURE_PIN);
        temperature += map(temperatureSample, 0, 0x3ff, 0, 0xfd);

        delay(SAMPLE_INTERVAL);
    }

    voltage /= SAMPLE_COUNT;
    current /= SAMPLE_COUNT;
    ambientLight /= SAMPLE_COUNT;
    opacity /= SAMPLE_COUNT;
    temperature /= SAMPLE_COUNT;

    // TODO: Should we fit both readings into less amount of bytes?
    // E.g. range of 0...126 for values?

    EEPROM.write(writeAddress++, voltage);
    EEPROM.write(writeAddress++, current);
    EEPROM.write(writeAddress++, ambientLight);
    EEPROM.write(writeAddress++, opacity);
    EEPROM.write(writeAddress++, temperature);
}

bool IsTimeToWrite(unsigned long currentTime) {
    if (writeTime == 0) {
        return true;
    }

    return currentTime > writeTime + MEASUREMENT_INTERVAL;
}

void outputReadings() {
    Serial.begin(9600);
    Serial.println("Printing previously recorded values.");
    Serial.println("Voltage (steps); Current (steps); Ambient light (steps); Opacity (steps); Temperature (steps)");

    int blocks = EEPROM.read(EEPROM_SIZE - 1);
    if (!blocks) {
        Serial.println("No previous recording found.");
        Serial.end();
        return;
    }

    int blockIndex = 0;
    int valueIndex = 0;
    int totalValues = 0;

    // Prints all measured quantities to one row. Which one is printed now?
    uint8_t quantityIndex = 0;

    for (int address = 0; address < EEPROM_SIZE - 1; address++) {
        byte value = EEPROM.read(address);
        if (value == WRITE_START_MARK) {
            Serial.print("Printing values from recording block ");
            Serial.println(blockIndex, DEC);

            totalValues += valueIndex;
            valueIndex = 0;
        }
        else if (value == WRITE_END_MARK) {
            Serial.print("Block ");
            Serial.print(blockIndex, DEC);
            Serial.print(" contained ");
            Serial.print(valueIndex, DEC);
            Serial.println(" values");

            blockIndex++;

            if (blockIndex == blocks) {
                break;
            }
        }
        else if (quantityIndex < QUANTITY_COUNT - 1) {
            Serial.print(value, DEC);
            Serial.print("; ");
            valueIndex++;

            quantityIndex++;
        }
        else {
            Serial.print(value, DEC);
            Serial.println();
            valueIndex++;

            quantityIndex = 0;
        }
    }

    totalValues += valueIndex;

    Serial.print("Done printing ");
    Serial.print(totalValues, DEC);
    Serial.print(" values from ");
    Serial.print(blockIndex, DEC);
    Serial.println(" blocks");
    Serial.end();
}

void NotifySolutionPumpTimer(unsigned long currentTime) {
    if (
        isSolutionPumpRunning
        && currentTime > solutionPumpChangeTime + SOLUTION_PUMP_ON_TIME
    ) {
        analogWrite(SOLUTION_PUMP_PIN, 0);
        ChangeSolutionPumpState(currentTime);
    }
    else if (
        !isSolutionPumpRunning
        && currentTime > solutionPumpChangeTime + SOLUTION_PUMP_OFF_TIME
    ) {
        analogWrite(SOLUTION_PUMP_PIN, SOLUTION_PUMP_PWM_DUTY_CYCLE);
        ChangeSolutionPumpState(currentTime);
    }
}

void ChangeSolutionPumpState(unsigned long currentTime) {
    isSolutionPumpRunning = !isSolutionPumpRunning;
    solutionPumpChangeTime = currentTime;
}
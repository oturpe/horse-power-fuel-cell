#include <EEPROM.h>

// Analog in pin sensing the voltage
#define V_SENSE_PIN A0
// Analog in pin sensing the current
#define I_SENSE_PIN A1
// Pin for button that outputs saved readings
#define CONTROL_BUTTON_PIN 2
// Pin for recording indicator
#define RECORDING_INDICATOR_PIN 5

// How the control button must be pressed to register a button press
#define BUTTON_OUTPUT_DELAY 200
// How the control button must be pressed to start or stop writing values.
// Given in units of millisecond
#define BUTTON_WRITE_DELAY 2000

// How many samples to average for each reading
#define SAMPLE_COUNT 20
// How long to wait between samples
#define SAMPLE_INTERVAL 100

// How often measurements are taken. Given in units of millisecond.
#define MEASUREMENT_INTERVAL (2*60*1000)

// Size of eeprom memory to read
#define EEPROM_SIZE 1024
// Marker to write for write start
#define WRITE_START_MARK 0xff
// Marker to write for write end
#define WRITE_END_MARK 0xfe

bool isWriting = false;

// Eeprom address to write
int writeAddress = 0;

// If control button is pressed
bool wasButtonDown = false;

// The moment of time when button went down
int buttonDownTime = 0;
// The moment values were last written
int writeTime = 0;

// How many blocks have been written
int blocksWritten = 0;

void setup() {
    analogReference(INTERNAL);

    pinMode(V_SENSE_PIN, INPUT);
    pinMode(I_SENSE_PIN, INPUT);
    pinMode(CONTROL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RECORDING_INDICATOR_PIN, OUTPUT);
}

void loop() {
    int currentTime = millis();
    
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
    if (writeAddress == EEPROM_SIZE -1) {
        return;
    }

    int voltage = 0;
    int current = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int voltageSample = analogRead(V_SENSE_PIN);
	    voltage += constrain(voltageSample, 0, 0xfd);

	    int currentSample = analogRead(I_SENSE_PIN);
        current += constrain(currentSample, 0, 0xfd);

        delay(SAMPLE_INTERVAL);
    }

    voltage /= SAMPLE_COUNT;
    current /= SAMPLE_COUNT;

    // TODO: Should we fit both readings into 1 byte?
    // So 0...126 for both values?

    EEPROM.write(writeAddress++, voltage);
    EEPROM.write(writeAddress++, current);
}

bool IsTimeToWrite(int currentTime) {
	if (writeTime == 0) {
	    return true;
	}

	return currentTime - writeTime > MEASUREMENT_INTERVAL;
}

void outputReadings() {
    Serial.begin(9600);
    Serial.println("Printing previously recorded values.");
    Serial.println("Values given in pairs (V, I)");

    int blocks = EEPROM.read(EEPROM_SIZE - 1);
    if (!blocks) {
        Serial.println("No previous recording found.");
        Serial.end();
        return;
    }

    int blockIndex = 0;
    int valueIndex = 0;
    int totalValues = 0;

    // Prints voltage and current on the same row. Which one are we
    // prining now?
    bool isVoltage = true;

    for (int address = 0; address < EEPROM_SIZE - 1; address++) {
        byte value = EEPROM.read(address);
        if (value == WRITE_START_MARK) {
            Serial.print("Printing values from recording block ");
            Serial.println(blockIndex++, DEC);

            totalValues += valueIndex;
            valueIndex = 0;
        }
        else if (value == WRITE_END_MARK) {
            Serial.print("Block ");
            Serial.print(blockIndex, DEC);
            Serial.print(" contained ");
            Serial.print(valueIndex, DEC);
            Serial.println(" values");
            
            if (blockIndex == blocks) {
                break;
            }
        }
        else {
            if (isVoltage) {
                Serial.print(value, DEC);
                Serial.print("; ");
                valueIndex++;

                isVoltage = !isVoltage;
            }
            else {
                Serial.print(value, DEC);
                Serial.println();
                valueIndex++;

                isVoltage = !isVoltage;
            }
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

void outputBlockEndMark(int blockIndex, int valueIndex) {
}
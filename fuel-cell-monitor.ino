// TODO:
//     Allow reading values from previous recoding
//     Prevent printing of values when trying to stop writing
#include <EEPROM.h>

// Analog in pin sensing the voltage
#define V_SENSE_PIN A0
// Analog in pin sensing the current
#define I_SENSE_PIN A1
// Pin for button that outputs saved readings
#define CONTROL_BUTTON_PIN 2
// Pin for recording indicator
#define RECORDING_INDICATOR_PIN 13

// How the control button must be pressed to start or stop writing values.
// Given in units of millisecond
#define WRITE_CONTROL_DELAY 2000

// How often values are saved. Given in units of millisecond.
#define WRITE_INTERVAL 1000 

// Size of eeprom memory to read
#define EEPROM_SIZE 1024
// Marker to write for write start
#define WRITE_START_MARK 0xff

bool isWriting = false;

// Eeprom address to write
int writeAddress = 0;

// If control button is pressed
bool wasButtonDown = false;

// The moment of time when button went down
int buttonDownTime = 0;
// The moment values were last written
int writeTime = 0;

void setup() {
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

        if (currentTime - buttonDownTime > WRITE_CONTROL_DELAY) {
            isWriting = !isWriting;
            if (isWriting) {
                 WriteStartMark();
            }
        } else {
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
    if (writeAddress == EEPROM_SIZE) {
        return;
    }

	EEPROM.write(writeAddress++, WRITE_START_MARK);
}

void WriteReadings() {
    if (writeAddress == EEPROM_SIZE) {
        return;
    }

    int voltage = analogRead(V_SENSE_PIN);
	voltage = constrain(voltage, 0, 0xfe);

	int current = analogRead(I_SENSE_PIN);
    current = constrain(current, 0, 0xfe);

    // TODO: Should we fit both readings into 1 byte?
    // So 0...126 for both values?

    EEPROM.write(writeAddress++, voltage);
    EEPROM.write(writeAddress++, current);
}

bool IsTimeToWrite(int currentTime) {
	if (writeTime == 0) {
	    return true;
	}

	return currentTime - writeTime > WRITE_INTERVAL;
}

void outputReadings() {
    Serial.begin(9600);

	for (int address = 0; address < writeAddress; address++) {
	    byte value = EEPROM.read(address);
	    if (value == WRITE_START_MARK) {
	        Serial.println("=== Write started ===");
	    } else {
	        Serial.print(value, DEC);
	        Serial.println();
	    }
	}
}
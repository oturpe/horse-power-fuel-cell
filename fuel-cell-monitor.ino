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

// Liquid crystal display data pins
#define SCREEN_DB0_PIN 9
#define SCREEN_DB1_PIN 2
#define SCREEN_DB2_PIN 8
#define SCREEN_DB3_PIN 3
#define SCREEN_DB4_PIN 7
#define SCREEN_DB5_PIN 4
#define SCREEN_DB6_PIN 6
#define SCREEN_DB7_PIN 5
// Liquid crystal display data/~command pin
#define SCREEN_DC_PIN 13
// Liquid crystal display enable pins
#define SCREEN_ENABLE1_PIN 10
#define SCREEN_ENABLE2_PIN A0
// Liquid crystal display reset pin
#define SCREEN_RESET_PIN 12

#include "screen.h"

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

// How often something is written to screen. Given in units of millisecond
#define SCREEN_UPDATE_PERIOD (5 * 1000L)

enum MeasuredQuantity {
    VOLTAGE = 0,
    CURRENT = 1,
    AMBIENT_LIGHT = 2,
    OPACITY = 3,
    TEMPERATURE =4
};

char * quantityName(uint8_t quantityIndex) {
    MeasuredQuantity quantity = (MeasuredQuantity)quantityIndex;

    switch (quantity) {
    case VOLTAGE:
        return "Voltage";
    case CURRENT:
        return "Current";
    case AMBIENT_LIGHT:
        return "Ambient light";
    case OPACITY:
        return "Opacity";
    case TEMPERATURE:
        return "Temperature";
    }
}

char * quantityUnit(uint8_t quantityIndex) {
    MeasuredQuantity quantity = (MeasuredQuantity)quantityIndex;

    switch (quantity) {
    case VOLTAGE:
        return "mV";
    case CURRENT:
        return "uA";
    case AMBIENT_LIGHT:
        return "lux";
    case OPACITY:
        return "units";
    case TEMPERATURE:
        return "C";
    }
}

uint32_t quantityInUnits(uint8_t quantityIndex, uint8_t steps) {
    MeasuredQuantity quantity = (MeasuredQuantity)quantityIndex;

    switch(quantity) {
    case VOLTAGE:
        // Return millivolts based on using internal voltage refernce.
        return map(steps, 0, 0xfe, 0, 1100);
    case CURRENT:
        // Return microamperes based on using 47 kiloohm load and 1 kilo sense resistance
        return map(steps, 0, 0xfe, 0, 1100);
    case AMBIENT_LIGHT:
        return map(steps, 0, 0xfe, 0, 1000);
    case OPACITY:
        return steps;
    case TEMPERATURE:
        return 0.25*steps;
    }
}

uint8_t nextQuantityIndex(uint8_t quantityIndex) {
    MeasuredQuantity quantity = (MeasuredQuantity)quantityIndex;

    switch (quantity) {
    case VOLTAGE:
        return CURRENT;
    case CURRENT:
        return AMBIENT_LIGHT;
    case AMBIENT_LIGHT:
        return OPACITY;
    case OPACITY:
        return TEMPERATURE;
    case TEMPERATURE:
        return VOLTAGE;
    }
}

// In-memory storage of latest readings.
uint8_t readings[QUANTITY_COUNT][SCREEN_DIMENSION_WIDTH];
// Pointer to the next reading to write.
uint8_t nextReadingIndex = 0;

bool isWriting = false;

// Eeprom address to write
int writeAddress = 0;

// If control button is pressed
bool wasButtonDown = false;

// The moment of time when button went down
unsigned long buttonDownTime = 0;
// The moment values were last written
unsigned long writeTime = 0;
// The moment when reading were last collected
unsigned long lastReadingTime = 0;

// If solution pump is running
bool isSolutionPumpRunning = false;
// When solution pump was started or stopped the last time
unsigned long solutionPumpChangeTime = 0;

// How many blocks have been written
int blocksWritten = 0;

// The last time the screen was updated. Given in units of millisecond
unsigned long screenLastUpdateTime = 0;

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

    pinMode(SCREEN_ENABLE1_PIN, OUTPUT);
    pinMode(SCREEN_ENABLE2_PIN, OUTPUT);
    pinMode(SCREEN_DC_PIN, OUTPUT);
    pinMode(SCREEN_RESET_PIN, OUTPUT);

    pinMode(SCREEN_DB0_PIN, OUTPUT);
    pinMode(SCREEN_DB1_PIN, OUTPUT);
    pinMode(SCREEN_DB2_PIN, OUTPUT);
    pinMode(SCREEN_DB3_PIN, OUTPUT);
    pinMode(SCREEN_DB4_PIN, OUTPUT);
    pinMode(SCREEN_DB5_PIN, OUTPUT);
    pinMode(SCREEN_DB6_PIN, OUTPUT);
    pinMode(SCREEN_DB7_PIN, OUTPUT);

    initializeScreen();
    //screenSetMapping(SCREEN_SEGMAP_INVERT);

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

    if (currentTime > lastReadingTime + SCREEN_UPDATE_PERIOD) {
        collectReadings();
        lastReadingTime = currentTime;
    }
    if (isWriting && isTimeToWrite(currentTime)) {
        WriteReadings();
        writeTime = currentTime;
    }

    NotifySolutionPumpTimer(currentTime);
    NotifyScreen(currentTime);
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

void collectReadings() {
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

    readings[(int)VOLTAGE][nextReadingIndex] = voltage / SAMPLE_COUNT;
    readings[(int)CURRENT][nextReadingIndex] = current / SAMPLE_COUNT;
    readings[(int)AMBIENT_LIGHT][nextReadingIndex] = ambientLight / SAMPLE_COUNT;
    readings[(int)OPACITY][nextReadingIndex] = opacity / SAMPLE_COUNT;
    readings[(int)TEMPERATURE][nextReadingIndex] = temperature / SAMPLE_COUNT;

    nextReadingIndex = (nextReadingIndex + 1) % SCREEN_DIMENSION_WIDTH;
}

void WriteReadings() {
    // Last bit is amount of blocks
    if (writeAddress == EEPROM_SIZE - 1) {
        return;
    }

    // TODO: Should we fit both readings into less amount of bytes?
    // E.g. range of 0...126 for values?

    EEPROM.write(writeAddress++, readings[(int)VOLTAGE][nextReadingIndex - 1]);
    EEPROM.write(writeAddress++, readings[(int)CURRENT][nextReadingIndex - 1]);
    EEPROM.write(writeAddress++, readings[(int)AMBIENT_LIGHT][nextReadingIndex - 1]);
    EEPROM.write(writeAddress++, readings[(int)OPACITY][nextReadingIndex - 1]);
    EEPROM.write(writeAddress++, readings[(int)TEMPERATURE][nextReadingIndex - 1]);
}

bool isTimeToWrite(unsigned long currentTime) {
    if (writeTime == 0) {
        return true;
    }

    return currentTime > writeTime + MEASUREMENT_WRITE_INTERVAL;
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

// TODO: Implement writing something sensible to screen.
uint8_t currentScreenByte = 0;

char * text = "Horse Power";
uint8_t textIndex = 0;
MeasuredQuantity currentScreenQuantity = VOLTAGE;

void printMeasurementHeader() {
    char * text = quantityName(currentScreenQuantity);
    while(*text) {
        writeScreenCharacter(*text);
        text++;
    }

    writeScreenCharacter(' ');
    uint8_t reading = readings[(int)currentScreenQuantity][nextReadingIndex - 1];
    String numberString(quantityInUnits(currentScreenQuantity, reading));
    char * number = (char *)numberString.c_str();
    while (*number) {
        writeScreenCharacter(*number);
        number++;
    }
    writeScreenCharacter(' ');

    text = quantityUnit(currentScreenQuantity);
    while(*text) {
        writeScreenCharacter(*text);
        text++;
    }
}

void printMeasurementGraph() {
    for (int chip = SCREEN_CHIP_COUNT - 1; chip >= 0; chip--) {
        currentScreenChip = chip;
        screenSelectChip(currentScreenChip);
        for (int column = SCREEN_INVERTED_FIRST_COLUMN; column < SCREEN_COLUMN_COUNT + SCREEN_INVERTED_FIRST_COLUMN; column++) {
            currentScreenColumn = column;
            screenSelectColumn(currentScreenColumn);
            for (int page = 2; page >= 0; page--) {
                currentScreenPage = page;
                screenSelectPage(currentScreenPage);

                uint8_t pixelColumn = column - SCREEN_INVERTED_FIRST_COLUMN;
                pixelColumn += (chip == 0) ? SCREEN_COLUMN_COUNT : 0;
                uint8_t columnReadingIndex = nextReadingIndex;
                columnReadingIndex += pixelColumn;
                columnReadingIndex %= SCREEN_DIMENSION_WIDTH;

                uint8_t reading = readings[(int)currentScreenQuantity][columnReadingIndex];
                uint8_t graphHeight = map(reading, 0, 0xfd, 0, 23);
                uint8_t pageGraphStart = page * 8;
                if (
                    (graphHeight < pageGraphStart)
                    || (graphHeight >= pageGraphStart + 8)
                ) {
                    writeScreenData(0x00);
                }
                else {
                    graphHeight -= pageGraphStart;
                    writeScreenData(1 << graphHeight);
                }
            }
        }
    }
}

void NotifyScreen(unsigned long currentTime) {
    if (currentTime <= screenLastUpdateTime + SCREEN_UPDATE_PERIOD) {
        return;
    }
    screenLastUpdateTime = currentTime;

    currentScreenQuantity = (MeasuredQuantity)nextQuantityIndex(currentScreenQuantity);

    // Clear and start from the beginning of screen
    clearScreen();
    currentScreenColumn = SCREEN_INVERTED_FIRST_COLUMN;
    screenSelectColumn(currentScreenColumn);
    currentScreenPage = 3;
    screenSelectPage(currentScreenPage);
    currentScreenChip = 1;
    screenSelectChip(currentScreenChip);

    printMeasurementHeader();
    printMeasurementGraph();
}

void ChangeSolutionPumpState(unsigned long currentTime) {
    isSolutionPumpRunning = !isSolutionPumpRunning;
    solutionPumpChangeTime = currentTime;
}

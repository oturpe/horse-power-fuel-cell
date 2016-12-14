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
#define SCREEN_UPDATE_PERIOD 100

// Number of columns in the screen controller.
#define SCREEN_COLUMN_COUNT 61
// Number of pages in the screen controller.
#define SCREEN_PAGE_COUNT 4
// Number of chips in the screen controller.
#define SCREEN_CHIP_COUNT 2

/// WG12232 command codes
#define SCREEN_CMD_RMW       0xE0
#define SCREEN_CMD_END_RMW   0xEE
#define SCREEN_CMD_SOFT_RST  0xE2

#define SCREEN_CMD_DISP_ON   0xAF
#define SCREEN_CMD_DISP_OFF  0xAE

#define SCREEN_CMD_MAP_SEG   0xA0

#define SCREEN_CMD_SET_SDL   0xC0
#define SCREEN_CMD_SEL_PAGE  0xB8
#define SCREEN_CMD_SEL_COL   0

#define SCREEN_DTM_RW_BIT    2
#define SCREEN_DTM_CD_BIT    1

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

// Current start Display Line of the screen
uint8_t screenStartDisplayLine;
// Current Column Address of the screen
uint8_t currentScreenColumn = 0;
// Current Page Address of the screen
uint8_t currentScreenPage = 0;
// Current chip of the screen
uint8_t currentScreenChip = 0;
// Enable pin of the current chip
uint8_t currentScreenChipPin;
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

    delay(10);
    // Initialization code adapted from the display's datasheet
    digitalWrite(SCREEN_RESET_PIN, LOW);
    delay(10);
    digitalWrite(SCREEN_RESET_PIN, HIGH);
    delay(10);
    screenEndRmw();
    setScreenStartDisplayLine(0);
    displayOn();
    screenSelectChip(currentScreenChip);
    screenSelectPage(currentScreenPage);

    analogWrite(SOLUTION_PUMP_PIN, 0);
}

void screenCommand(uint8_t command) {
    digitalWrite(SCREEN_ENABLE1_PIN, HIGH);
    digitalWrite(SCREEN_ENABLE2_PIN, HIGH);
    // Set command mode
    digitalWrite(SCREEN_DC_PIN, LOW);
    delayMicroseconds(10);
    setScreenDataBits(command);
    digitalWrite(SCREEN_ENABLE1_PIN, LOW);
    digitalWrite(SCREEN_ENABLE2_PIN, LOW);
}

void writeScreenData(uint8_t character) {
    // Set data mode
    digitalWrite(currentScreenChipPin, HIGH);
    digitalWrite(SCREEN_DC_PIN, HIGH);
    // ???
    delayMicroseconds(10);
    setScreenDataBits(character);
    digitalWrite(currentScreenChipPin, LOW);
}

void setScreenDataBits(uint8_t bits) {
    digitalWrite(SCREEN_DB0_PIN, bits & 0x01);
    digitalWrite(SCREEN_DB1_PIN, bits & 0x02);
    digitalWrite(SCREEN_DB2_PIN, bits & 0x04);
    digitalWrite(SCREEN_DB3_PIN, bits & 0x08);
    digitalWrite(SCREEN_DB4_PIN, bits & 0x10);
    digitalWrite(SCREEN_DB5_PIN, bits & 0x20);
    digitalWrite(SCREEN_DB6_PIN, bits & 0x40);
    digitalWrite(SCREEN_DB7_PIN, bits & 0x80);
}

/**
 * To switch display on for currently selected chip.
 */
inline void displayOn() {
    screenCommand(SCREEN_CMD_DISP_ON);
}

/**
 * To set Start Display Line.
 * \param line Start Display Line number (0..31).
 */
inline void setScreenStartDisplayLine(uint8_t line) {
    // TODO: How to save the line?
    screenStartDisplayLine = line;
    screenCommand(SCREEN_CMD_SET_SDL + line);
}

/**
 * To start Read/Modify/Write sequence.
 */
inline void screenBeginRmw() {
    screenCommand(SCREEN_CMD_RMW);
}

/**
 * To finish Read/Modify/Write sequence.
 */
inline void screenEndRmw() {
    screenCommand(SCREEN_CMD_END_RMW);
}


/**
 * To select page. 32 lines are segmented for 4 pages 8 lines each.
 * \param line Page number (0..3).
 */
inline void screenSelectPage(uint8_t page) {
    screenCommand(SCREEN_CMD_SEL_PAGE + page);
}

/**
 * Select the given chip for screen.
 */
inline void screenSelectChip(uint8_t chip) {
    switch (chip) {
    case 0:
        currentScreenChipPin = SCREEN_ENABLE1_PIN;
        break;
    case 1:
        currentScreenChipPin = SCREEN_ENABLE2_PIN;
        break;
    }
}

/**
 * To select column. 122 columns are controled by 2 chips 61 column each.
 * \param line Column number (0..60).
 */
inline void screenSelectColumn(uint8_t col) {
    screenCommand(SCREEN_CMD_SEL_COL + col);
}

inline static uint8_t screenColumnFromX(uint8_t x) {
    return (x > 60) ? x-61 : x;
}

uint8_t screenPageFromY(uint8_t y)
{
  uint8_t line = screenStartDisplayLine + y;
  if(line > 31)
    line -= 32;
  return line / 8;
}

uint8_t screenBitFromY(uint8_t y)
{
  uint8_t line = screenStartDisplayLine + y;
  if(line > 31)
    line -= 32;
  return line % 8;
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

// TODO: Implement writing something sensible to screen.
uint8_t currentScreenByte = 0;

void NotifyScreen(unsigned long currentTime) {
//    if (currentTime <= screenLastUpdateTime + SCREEN_UPDATE_PERIOD) {
//        return;
//    }
//    screenLastUpdateTime = currentTime;

    writeScreenData(currentScreenByte);
    currentScreenByte = currentScreenByte + 1;
    currentScreenColumn = (currentScreenColumn + 1) % SCREEN_COLUMN_COUNT;
    if (currentScreenColumn == 0) {
        screenSelectColumn(0);
        currentScreenPage = (currentScreenPage + 1) % SCREEN_PAGE_COUNT;
        screenSelectPage(currentScreenPage);
        if (currentScreenPage == 0) {
            currentScreenChip = (currentScreenChip + 1) % SCREEN_CHIP_COUNT;
            screenSelectChip(currentScreenChip);
        }
    }
}

void ChangeSolutionPumpState(unsigned long currentTime) {
    isSolutionPumpRunning = !isSolutionPumpRunning;
    solutionPumpChangeTime = currentTime;
}

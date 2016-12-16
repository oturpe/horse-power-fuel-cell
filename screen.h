#include <Arduino.h>

#include "screen-ascii-maps.h"
// Control for NHD-12232AZ-FL-YBW liquid crystal display screen.
//
// Microcontroller pins have to defined using by following definitions
// before including this file:
//
// Liquid crystal display data pins
//    #define SCREEN_DB0_PIN 9
//    #define SCREEN_DB1_PIN 2
//    #define SCREEN_DB2_PIN 8
//    #define SCREEN_DB3_PIN 3
//    #define SCREEN_DB4_PIN 7
//    #define SCREEN_DB5_PIN 4
//    #define SCREEN_DB6_PIN 6
//    #define SCREEN_DB7_PIN 5
// Liquid crystal display data/~command pin
//    #define SCREEN_DC_PIN 13
// Liquid crystal display enable pins
//    #define SCREEN_ENABLE1_PIN 10
//    #define SCREEN_ENABLE2_PIN A0
// Liquid crystal display reset pin
//    #define SCREEN_RESET_PIN 12

// First column in the screen when inverting
#define SCREEN_INVERTED_FIRST_COLUMN 19
// Number of columns in the screen controller.
#define SCREEN_COLUMN_COUNT 61
// Number of pages in the screen controller.
#define SCREEN_PAGE_COUNT 4
// Number of chips in the screen controller.
#define SCREEN_CHIP_COUNT 2

// Width of screen. Given  in units of pixel
#define SCREEN_DIMENSION_WIDTH 122

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

/// Segment mapping
#define SCREEN_SEGMAP_NORMAL       0
#define SCREEN_SEGMAP_INVERT       1

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
 * To set segment mapping for currently selected chip.
 * \param seg Segment mapping mode. SCREEN_SEGMAP_NORMAL or SCREEN_SEGMAP_INVERT.
 */
inline void screenSetMapping(uint8_t seg) {
    screenCommand(SCREEN_CMD_MAP_SEG + seg);
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

void writeScreenData(uint8_t data) {
    // Set data mode
    digitalWrite(currentScreenChipPin, HIGH);
    digitalWrite(SCREEN_DC_PIN, HIGH);
    // ???
    delayMicroseconds(10);
    setScreenDataBits(data);
    digitalWrite(currentScreenChipPin, LOW);
}

void writeScreenCharacter(uint8_t character) {
    //uint8_t * columns = screenAsciiMaps[character-32];
    uint8_t * columns = screenInvertedAsciiMap(character - 32);
    for (int i = 0; i < SCREEN_CHARACTER_WIDTH; i++) {
        writeScreenData(columns[i]);

        currentScreenColumn = (currentScreenColumn + 1) % (SCREEN_COLUMN_COUNT + SCREEN_INVERTED_FIRST_COLUMN);
        if (currentScreenColumn == 0) {
            currentScreenColumn = SCREEN_INVERTED_FIRST_COLUMN;
            screenSelectColumn(currentScreenColumn);
            currentScreenChip -= 1;
            if (currentScreenChip >= SCREEN_CHIP_COUNT) {
            currentScreenChip = SCREEN_CHIP_COUNT - 1;
            }

            screenSelectChip(currentScreenChip);
            if (currentScreenChip == SCREEN_CHIP_COUNT - 1) {
                currentScreenPage -= 1;
                if (currentScreenPage >= SCREEN_PAGE_COUNT) {
                     currentScreenPage = SCREEN_PAGE_COUNT -1;
                }
                screenSelectPage(currentScreenPage);
            }
        }
    }
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

void clearScreen() {
    for (int page = 0; page < SCREEN_PAGE_COUNT; page++) {
        currentScreenPage = page;
        screenSelectPage(page);
        for (int chip = 0; chip < SCREEN_CHIP_COUNT; chip++) {
            currentScreenChip = chip;
            screenSelectChip(chip);
            for (int column = SCREEN_INVERTED_FIRST_COLUMN; column < SCREEN_COLUMN_COUNT + SCREEN_INVERTED_FIRST_COLUMN; column++) {
                writeScreenData(0x00);
            }
            screenSelectColumn(SCREEN_INVERTED_FIRST_COLUMN);
        }
    }
}

void initializeScreen() {
    delay(10);
    // Initialization code adapted from the display's datasheet
    digitalWrite(SCREEN_RESET_PIN, LOW);
    delay(10);
    digitalWrite(SCREEN_RESET_PIN, HIGH);
    delay(10);
    screenEndRmw();
    setScreenStartDisplayLine(0);
    screenSetMapping(SCREEN_SEGMAP_INVERT);
    displayOn();
    clearScreen();
    screenSelectChip(currentScreenChip);
    screenSelectPage(currentScreenPage);
}

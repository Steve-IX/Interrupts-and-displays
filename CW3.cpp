#include "MicroBit.h"

// Extern declarations for objects defined elsewhere
extern NRF52I2C i2cInt;  // Internal I2C (accelerometer)
extern NRF52I2C i2cExt;  // External I2C (OLED)
extern NRF52Serial serial;

// --- Pin Definitions for Micro:bit LED Matrix ---
#define ROW_1_PIN 21
#define ROW_2_PIN 22
#define ROW_3_PIN 15
#define ROW_4_PIN 24
#define ROW_5_PIN 19

#define COL_1_PIN 28
#define COL_2_PIN 11
#define COL_3_PIN 31
#define COL_4_PIN 5
#define COL_5_PIN 30

#define NUM_ROWS 5
#define NUM_COLS 5

static uint8_t microBitDisplayFrameBuffer[NUM_ROWS][NUM_COLS] = {0};
static uint8_t currentRow = 0;

static void driveRow(uint8_t row);
static void driveColumns(uint8_t row);
static void microBitDisplayIsr(void);

// Initialize the LED Matrix Display
void initMicroBitDisplay() {
    // Configure GPIO for rows and columns
    NRF_P0->DIRSET = (1 << ROW_1_PIN) | (1 << ROW_2_PIN) | (1 << ROW_3_PIN) |
                     (1 << ROW_4_PIN) | (1 << ROW_5_PIN);
    NRF_P0->DIRSET = (1 << COL_1_PIN) | (1 << COL_2_PIN) | (1 << COL_3_PIN) |
                     (1 << COL_5_PIN);
    NRF_P1->DIRSET = (1 << COL_4_PIN);

    memset(microBitDisplayFrameBuffer, 0, sizeof(microBitDisplayFrameBuffer));

    // Configure TIMER2 for periodic interrupts (100 Hz refresh)
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER2->PRESCALER = 4; // 1 MHz
    NRF_TIMER2->CC[0] = 10000 / NUM_ROWS;
    NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

    NVIC_SetVector(TIMER2_IRQn, (uint32_t)microBitDisplayIsr);
    NVIC_EnableIRQ(TIMER2_IRQn);

    NRF_TIMER2->TASKS_START = 1;
}

// Clear the LED Matrix Display
void clearMicroBitDisplay() {
    memset(microBitDisplayFrameBuffer, 0, sizeof(microBitDisplayFrameBuffer));
}

// Set a pixel on the LED Matrix
void setMicroBitPixel(uint8_t x, uint8_t y) {
    if (x < NUM_COLS && y < NUM_ROWS) {
        microBitDisplayFrameBuffer[y][x] = 1;
    }
}

// Clear a pixel on the LED Matrix
void clearMicroBitPixel(uint8_t x, uint8_t y) {
    if (x < NUM_COLS && y < NUM_ROWS) {
        microBitDisplayFrameBuffer[y][x] = 0;
    }
}

// Drive a specific row on the LED Matrix
static void driveRow(uint8_t row) {
    NRF_P0->OUTCLR = (1 << ROW_1_PIN) | (1 << ROW_2_PIN) | (1 << ROW_3_PIN) |
                     (1 << ROW_4_PIN) | (1 << ROW_5_PIN);

    switch (row) {
        case 0: NRF_P0->OUTSET = (1 << ROW_1_PIN); break;
        case 1: NRF_P0->OUTSET = (1 << ROW_2_PIN); break;
        case 2: NRF_P0->OUTSET = (1 << ROW_3_PIN); break;
        case 3: NRF_P0->OUTSET = (1 << ROW_4_PIN); break;
        case 4: NRF_P0->OUTSET = (1 << ROW_5_PIN); break;
    }
}

// Drive the columns for a specific row on the LED Matrix
static void driveColumns(uint8_t row) {
    NRF_P0->OUTSET = (1 << COL_1_PIN) | (1 << COL_2_PIN) |
                     (1 << COL_3_PIN) | (1 << COL_5_PIN);
    NRF_P1->OUTSET = (1 << COL_4_PIN);

    for (int col = 0; col < NUM_COLS; col++) {
        if (microBitDisplayFrameBuffer[row][col]) {
            switch (col) {
                case 0: NRF_P0->OUTCLR = (1 << COL_1_PIN); break;
                case 1: NRF_P0->OUTCLR = (1 << COL_2_PIN); break;
                case 2: NRF_P0->OUTCLR = (1 << COL_3_PIN); break;
                case 3: NRF_P1->OUTCLR = (1 << COL_4_PIN); break;
                case 4: NRF_P0->OUTCLR = (1 << COL_5_PIN); break;
            }
        }
    }
}

// ISR for TIMER2 to refresh the LED Matrix
static void microBitDisplayIsr(void) {
    if (NRF_TIMER2->EVENTS_COMPARE[0]) {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;
        driveRow(currentRow);
        driveColumns(currentRow);
        currentRow = (currentRow + 1) % NUM_ROWS;
    }
}

//subtask 2 
// --- OLED code ---
#define SSD1306_I2C_ADDRESS 0x3C
static uint8_t oledDisplayFrameBuffer[8][128] = {0};

static void oledSendCommand(uint8_t command) {
    uint8_t buffer[2] = {0x00, command};
    i2cExt.write(SSD1306_I2C_ADDRESS << 1, buffer, 2);
}

static void updateOledDisplay(void) {
    for (uint8_t page = 0; page < 8; page++) {
        oledSendCommand(0xB0 + page);
        oledSendCommand(0x00);
        oledSendCommand(0x10);

        uint8_t buffer[129] = {0x40};
        memcpy(&buffer[1], oledDisplayFrameBuffer[page], 128);
        i2cExt.write(SSD1306_I2C_ADDRESS << 1, buffer, 129);
    }
}

void clearOledDisplay(void) {
    memset(oledDisplayFrameBuffer, 0, sizeof(oledDisplayFrameBuffer));
    updateOledDisplay();
}

void initOledDisplay(void) {
    // set i2c speed to 400kHz
    i2cExt.setFrequency(400000);
    uint8_t commands[] = {
        0x80, 0xAE,
        0x80, 0xD5, 0x80,
        0x80, 0xA8, 0x3F,
        0x80, 0xD3, 0x00,
        0x80, 0x40,
        0x80, 0x8D, 0x14,
        0x80, 0x20, 0x00,
        0x80, 0xA1,
        0x80, 0xC8,
        0x80, 0xDA, 0x12,
        0x80, 0x81, 0x7F,
        0x80, 0xD9, 0xF1,
        0x80, 0xDB, 0x40,
        0x80, 0xA4,
        0x80, 0xA6,
        0x80, 0xAF
    };
    i2cExt.write(SSD1306_I2C_ADDRESS << 1, commands, sizeof(commands));
    clearOledDisplay();
}

//sets the oled dispay per pixel
void setOledPixel(uint8_t x, uint8_t y) {
    if (x < 128 && y < 64) {
        uint8_t page = y / 8;
        uint8_t bit = 1 << (y % 8);
        oledDisplayFrameBuffer[page][x] |= bit;
    }
}

// clears the oled display
void clearOledPixel(uint8_t x, uint8_t y) {
    if (x < 128 && y < 64) {
        uint8_t page = y / 8;
        uint8_t bit = 1 << (y % 8);
        oledDisplayFrameBuffer[page][x] &= ~bit;
    }
}

//draw the Oled lines 
void drawOledLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
    int dx = abs(x_end - x_start);
    int dy = abs(y_end - y_start);
    int sx = (x_start < x_end) ? 1 : -1;
    int sy = (y_start < y_end) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        setOledPixel(x_start, y_start);
        if (x_start == x_end && y_start == y_end) {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x_start += sx; }
        if (e2 < dx) { err += dx; y_start += sy; }
    }
    updateOledDisplay();
}

// --- Subtask 3 ---
#define BUTTON_A_PIN 14
#define BUTTON_B_PIN 23
#define ACCELEROMETER_I2C_ADDRESS 0x19
#define WHO_AM_I_REG 0x0F
#define OUT_X_L_A 0x28

enum GraphMode {
    ACCELERATION_MODE,
    JERK_MODE
};

static volatile GraphMode currentMode = ACCELERATION_MODE;
static volatile bool timerTriggered = false;
static bool accelerometerInitialized = false;

// Button handling via GPIOTE
static volatile bool buttonA_pressed = false;
static volatile bool buttonB_pressed = false;


//write the registers 
bool writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return (i2cInt.write(ACCELEROMETER_I2C_ADDRESS << 1, buffer, 2) == 0);
}

uint8_t readRegister(uint8_t reg) {
    i2cInt.write(ACCELEROMETER_I2C_ADDRESS << 1, &reg, 1, true);
    uint8_t value = 0;
    i2cInt.read(ACCELEROMETER_I2C_ADDRESS << 1, &value, 1);
    return value;
}

//get the acceleromter axis
int16_t getAccelerometerAxis(uint8_t lowReg) {
    uint8_t lowByte = readRegister(lowReg);
    uint8_t highByte = readRegister(lowReg + 1);
    int16_t rawVal = ((int16_t)((highByte << 8) | lowByte)) >> 6;
    return rawVal;
}

int16_t getAccelerometerX() {
    return getAccelerometerAxis(OUT_X_L_A);
}

void initAccelerometer() {
    if (!accelerometerInitialized) {
        i2cInt.setFrequency(100000); // 100kHz
        uint8_t whoAmI = readRegister(WHO_AM_I_REG);
        if (whoAmI != 0x33) {
            serial.printf("Accelerometer not found!\r\n");
            return;
        }

        // CTRL_REG1_A: 100Hz data rate, all axes enabled
        // CTRL_REG4_A: +/- 2g full scale
        writeRegister(0x20, 0x57);
        writeRegister(0x23, 0x00);
        accelerometerInitialized = true;
    }
}

int16_t mapValue(int16_t value, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax) {
    return (int32_t)(value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

//scroll display
void scrollDisplay() {
    for (uint8_t page = 0; page < 8; page++) {
        for (uint8_t col = 0; col < 127; col++) {
            oledDisplayFrameBuffer[page][col] = oledDisplayFrameBuffer[page][col + 1];
        }
        oledDisplayFrameBuffer[page][127] = 0;
    }
}

//plot the acceleration
void plotAcceleration(uint8_t x, int16_t accelData) {
    uint8_t height = (uint8_t)mapValue(accelData, -512, 511, 0, 63);
    for (uint8_t y = 63; y >= (63 - height); y--) {
        setOledPixel(x, y);
        if (y == 0) break; // Avoid underflow of uint8_t
    }
}

//blot jerk
void plotJerk(uint8_t x, int16_t jerkData) {
    setOledPixel(x, 31); // baseline pixel
    int8_t length = (int8_t)mapValue(jerkData, -1023, 1023, -31, 31);
    if (length > 0) {
        for (int8_t y = 30; y >= (31 - length); y--) {
            setOledPixel(x, y);
        }
    } else if (length < 0) {
        for (int8_t y = 32; y <= (31 - length); y++) {
            setOledPixel(x, y);
        }
    }
}

// Timer ISR for refreshing at given rate
static void timerISR(void) {
    if (NRF_TIMER3->EVENTS_COMPARE[0]) {
        NRF_TIMER3->EVENTS_COMPARE[0] = 0;
        timerTriggered = true;
    }
}

// Setup a timer for refreshRate
void initTimer(uint16_t refreshRate) {
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER3->PRESCALER = 4; 
    NRF_TIMER3->CC[0] = 1000000 / refreshRate; 
    NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

    NVIC_SetVector(TIMER3_IRQn, (uint32_t)timerISR);
    NVIC_EnableIRQ(TIMER3_IRQn);

    NRF_TIMER3->TASKS_START = 1;
}

// Configure GPIOTE for button interrupts
static void GPIOTE_IRQHandler(void) {
    if ((NRF_GPIOTE->EVENTS_PORT) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_PORT_Msk)) {
        // Clear the event
        NRF_GPIOTE->EVENTS_PORT = 0;

        // Check button states (active low)
        bool a_pressed = ((NRF_P0->IN & (1 << BUTTON_A_PIN)) == 0);
        bool b_pressed = ((NRF_P0->IN & (1 << BUTTON_B_PIN)) == 0);

        if (a_pressed) {
            buttonA_pressed = true;
        }

        if (b_pressed) {
            buttonB_pressed = true;
        }
    }
}

// Setup GPIOTE for button interrupts
void initButtonsWithInterrupts() {
    // Configure Button A pin (active low, sense on low level)
    NRF_P0->PIN_CNF[BUTTON_A_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                    (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                    (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

    // Configure Button B pin (active low, sense on low level)
    NRF_P0->PIN_CNF[BUTTON_B_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                    (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                    (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

    // Enable GPIOTE PORT events
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_SetVector(GPIOTE_IRQn, (uint32_t)GPIOTE_IRQHandler);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

// Update LED matrix to reflect current mode
void updateLedModeIndicator() {
    clearMicroBitDisplay();

    // Middle pixel: (2,2)
    setMicroBitPixel(2,2);

    if (currentMode == ACCELERATION_MODE) {
        // Light up the leftmost column plus the center pixel
        for (uint8_t row = 0; row < 5; row++) {
            setMicroBitPixel(0, row);
        }
    } else {
        // Light up the rightmost column plus the center pixel
        for (uint8_t row = 0; row < 5; row++) {
            setMicroBitPixel(4, row);
        }
    }
}

// Main graphing function
void graphData(uint8_t refreshRate) {
    serial.printf("graphData: Starting with refreshRate=%d\r\n", refreshRate);

    initOledDisplay();
    initAccelerometer();
    initTimer(refreshRate);
    initMicroBitDisplay();
    initButtonsWithInterrupts();

    clearOledDisplay();
    int16_t prevAccel = getAccelerometerX();
    uint8_t x = 0;
    currentMode = ACCELERATION_MODE;
    updateLedModeIndicator();

    while (true) {
        // Handle mode switching via interrupts (no polling)
        if (buttonA_pressed) {
            buttonA_pressed = false;
            currentMode = ACCELERATION_MODE;
            clearOledDisplay();
            x = 0;
            prevAccel = getAccelerometerX();
            updateLedModeIndicator();
        }

        if (buttonB_pressed) {
            buttonB_pressed = false;
            currentMode = JERK_MODE;
            clearOledDisplay();
            x = 0;
            prevAccel = getAccelerometerX();
            updateLedModeIndicator();
        }

        if (timerTriggered) {
            timerTriggered = false;

            int16_t accelX = getAccelerometerX();
            if (currentMode == ACCELERATION_MODE) {
                plotAcceleration(x, accelX);
            } else {
                int16_t jerk = accelX - prevAccel;
                plotJerk(x, jerk);
            }

            prevAccel = accelX;

            if (x >= 127) {
                scrollDisplay();
            } else {
                x++;
            }

            updateOledDisplay();
        }
    }
}
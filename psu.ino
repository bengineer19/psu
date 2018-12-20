/*
 * Code for diy PSU from audio amp by Ben James.
 * Target: Arduino MEGA 2560
 *
 * UART manual for buck converter:
 * http://www.cs.droking.com/support/topic/200310-dc-dc-buck-converter-uart/
 */


#include <Elegoo_GFX.h>    // Core graphics library for TFT
#include <Elegoo_TFTLCD.h> // Hardware-specific TFT library
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <BuckPSU.h>


/*
 * LCD Pinout
 * ==========
 *
 * Control pins are on Analog pins since apparently this is required for using
 * the touch-screen(...?)
 * LCD_RESET  => A4 (Can alternately just connect to Arduino's reset pin)
 * LCD_CS     => A3
 * LCD_CD/RS  => A2
 * LCD_WR     => A1
 * LCD_RD     => A0
 *
 * Digital pins follow shield connections
 * D0 => 8  (Notice these are
 * D1 => 9   NOT in order!)
 * D2 => 2
 * D3 => 3
 * D4 => 4
 * D5 => 5
 * D6 => 6
 * D7 => 7
 *
 *                  320 px
 *     |-------------------------------------
 *     |
 *     |  VVVVVVVVVV           vvvvvvvv
 *     |  VVVVVVVVVV           vvvvvvvv
 *240px|
 *     |  IIIIIIIIII
 *     |  IIIIIIIIII
 *     |
 *     |
 *     |-------------------------------------
 */
#define LCD_CS    A3
#define LCD_CD    A2
#define LCD_WR    A1
#define LCD_RD    A0
#define LCD_RESET A4

#define TFT_IDENTIFIER 0x9341

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Define UI geometry
#define SMPS_VOLT_START_X           40
#define SMPS_VOLT_START_Y           100
#define SMPS_VOLT_TEXT_SIZE         3
#define SMPS_VOLT_CHAR_WIDTH        (5 * SMPS_VOLT_TEXT_SIZE)
#define SMPS_VOLT_CHAR_HEIGHT       (7 * SMPS_VOLT_TEXT_SIZE)
#define SMPS_VOLT_CHAR_SPACER       2
#define SMPS_VOLT_TOTAL_CHAR_WIDTH  (SMPS_VOLT_CHAR_WIDTH + SMPS_VOLT_CHAR_SPACER)
#define SMPS_VOLT_NUM_CHARS         5

#define SMPS_VSET_START_X           15
#define SMPS_VSET_START_Y           50
#define SMPS_VSET_TEXT_SIZE         4
#define SMPS_VSET_CHAR_WIDTH        (5 * SMPS_VSET_TEXT_SIZE)
#define SMPS_VSET_CHAR_HEIGHT       (7 * SMPS_VSET_TEXT_SIZE)
#define SMPS_VSET_CHAR_SPACER       2
#define SMPS_VSET_TOTAL_CHAR_WIDTH  (SMPS_VSET_CHAR_WIDTH + SMPS_VSET_CHAR_SPACER)
#define SMPS_VSET_NUM_CHARS         5

#define SMPS_CURR_START_X           15
#define SMPS_CURR_START_Y           160
#define SMPS_CURR_TEXT_SIZE         4
#define SMPS_CURR_CHAR_WIDTH        (5 * SMPS_CURR_TEXT_SIZE)
#define SMPS_CURR_CHAR_HEIGHT       (7 * SMPS_CURR_TEXT_SIZE)
#define SMPS_CURR_CHAR_SPACER       2
#define SMPS_CURR_TOTAL_CHAR_WIDTH  (SMPS_CURR_CHAR_WIDTH + SMPS_CURR_CHAR_SPACER)
#define SMPS_CURR_NUM_CHARS         5

#define SMPS_CSET_START_X           15
#define SMPS_CSET_START_Y           160
#define SMPS_CSET_TEXT_SIZE         4
#define SMPS_CSET_CHAR_WIDTH        (5 * SMPS_CSET_TEXT_SIZE)
#define SMPS_CSET_CHAR_HEIGHT       (7 * SMPS_CSET_TEXT_SIZE)
#define SMPS_CSET_CHAR_SPACER       2
#define SMPS_CSET_TOTAL_CHAR_WIDTH  (SMPS_CSET_CHAR_WIDTH + SMPS_CSET_CHAR_SPACER)
#define SMPS_CSET_NUM_CHARS         5

#define LINEAR_VOLT_START_X           185
#define LINEAR_VOLT_START_Y           50
#define LINEAR_VOLT_TEXT_SIZE         4
#define LINEAR_VOLT_CHAR_WIDTH        (5 * LINEAR_VOLT_TEXT_SIZE)
#define LINEAR_VOLT_CHAR_HEIGHT       (7 * LINEAR_VOLT_TEXT_SIZE)
#define LINEAR_VOLT_CHAR_SPACER       2
#define LINEAR_VOLT_TOTAL_CHAR_WIDTH  (LINEAR_VOLT_CHAR_WIDTH + LINEAR_VOLT_CHAR_SPACER)
#define LINEAR_VOLT_NUM_CHARS         4

#define ZONE_WIDTH      143
#define ZONE_HEIGHT     200
#define SMPS_ZONE_X     10
#define SMPS_ZONE_Y     40
#define LINEAR_ZONE_X   170
#define LINEAR_ZONE_Y   40

/*
 * Rotary encoders
 * ===============
 *
 * Voltage encoder is on A8 to A10:
 * CLK  => A8
 * DT   => A9
 * BTN  => A10
 *
 * Current encoder is on 10 to 12:
 * CLK  => 10
 * DT   => 11
 * BTN  => 12
 */
#define ENC_VOLT_CLK A8
#define ENC_VOLT_DT  A9
// Note that the button is currently not used due the way the way the ISR
// services both of the encoders
#define ENC_VOLT_BTN A10

#define ENC_CURR_CLK 10
#define ENC_CURR_DT  11
#define ENC_CURR_BTN 12

// Define how much parameters should change for each click of the rotary encs
#define MILLIVOLTS_PER_NOTCH_SLOW   100
#define MILLIVOLTS_PER_NOTCH_FAST   1000
#define MILLIAMPS_PER_NOTCH         100

// Scale ADC input values according the attenuation by the voltage dividers
#define ADC_0_1_DIVIDER_GAIN        7.73
#define ADC_2_3_DIVIDER_GAIN        11.15
#define ADC_ADDRESS                 0x48
#define ADC_MULTIPLIER              0.1875

#define MIN_SMPS_VOLTAGE_MILLIVOLTS 0
#define MAX_SMPS_VOLTAGE_MILLIVOLTS 40000

#define MIN_SMPS_CURRENT_MILLIAMPS  0
#define MAX_SMPS_CURRENT_MILLIAMPS  10000 // TBD

// When writing to the PSU, it is not possible to write to it again immediately
// after - it does not respond. It needs this delay to digest the message.
#define PSU_WRITE_DELAY_MS 300

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

ClickEncoder *voltEncoder;
ClickEncoder *currEncoder;

BuckPSU psu(Serial1);

Adafruit_ADS1115 ads(ADC_ADDRESS);

uint16_t millivoltsSMPS = 1000, prevMillivoltsSMPS = 0;
uint16_t milliampsSMPS = 100, prevMilliampsSMPS = 0;

int16_t voltEncValue, currEncValue;

void setup(void) {
    Serial.begin(9600);
    // Begin serial port for Buck comms
    Serial1.begin(4800);

    // Setup rotary encoders
    voltEncoder = new ClickEncoder(ENC_VOLT_CLK, ENC_VOLT_DT, ENC_VOLT_BTN, 4);
    currEncoder = new ClickEncoder(ENC_CURR_CLK, ENC_CURR_DT, ENC_CURR_BTN, 4);

    // Setup ISR to service encoder updater routines
    Timer1.initialize(1000);
    Timer1.attachInterrupt(encoderISR);

    // Setup ADC
    ads.begin();

    tft.reset();
    tft.begin(TFT_IDENTIFIER);
    // Landscape orientation
    tft.setRotation(3);
    tft.fillScreen(BLACK);
    setupUI();

//    setUIVoltageSMPS(millivoltsSMPS);
//    setUICurrentSMPS(milliampsSMPS);
    psu.setVoltageMilliVolts(millivoltsSMPS);
    psu.setCurrentMilliAmps(milliampsSMPS);
    psu.enableOutput(true);
    setUIVoltageLinear(0);
}

void loop()
{
   // Sample ADC and update linear voltage
   // setUIVoltageLinear(getLinearVoltageMillivolts());
//    Serial.println(psu.readVoltageMilliVolts());
    setUIVoltageActualSMPS(psu.readVoltageMilliVolts());
    delay(PSU_WRITE_DELAY_MS);
//    setUICurrentSMPS(psu.readCurrentMilliAmps());
//    delay(PSU_WRITE_DELAY_MS);


   millivoltsSMPS = updateMillivoltsSMPS(millivoltsSMPS);
   if (millivoltsSMPS != prevMillivoltsSMPS) {
       prevMillivoltsSMPS = millivoltsSMPS;

      setUIVoltageSetSMPS(millivoltsSMPS);
       psu.setVoltageMilliVolts(millivoltsSMPS);
       delay(PSU_WRITE_DELAY_MS);
   }
/*
   milliampsSMPS = updateMilliampsSMPS(milliampsSMPS);
   if (milliampsSMPS != prevMilliampsSMPS) {
       prevMilliampsSMPS = milliampsSMPS;

       // setUICurrentSMPS(milliampsSMPS);
       psu.setCurrentMilliAmps(milliampsSMPS);
       delay(PSU_WRITE_DELAY_MS);
   }
*/
}

// Update SMPS millivolts setting from the latest encoder reading
uint16_t updateMillivoltsSMPS(uint16_t millivolts){
    static bool voltAdjustSlow = false;

    // If button is clicked, change speed from slow to fast or vice versa
    ClickEncoder::Button voltEncoderBtn = voltEncoder->getButton();
    if(voltEncoderBtn == ClickEncoder::Clicked){
        voltAdjustSlow = !voltAdjustSlow;
    }
    // Set voltage adjust speed
    if(voltAdjustSlow){
        voltEncValue = voltEncoder->getValue() * MILLIVOLTS_PER_NOTCH_SLOW;
    }
    else{
        voltEncValue = voltEncoder->getValue() * MILLIVOLTS_PER_NOTCH_FAST;
    }
    // Guard against overflow of uint
    // (can occur when millivolts = 0, or when millivolts = 0.01V and
    // voltEncValue = -0.1V
    if((millivolts != 0 && abs(voltEncValue) <= millivolts) || (voltEncValue > 0)){
        millivolts += voltEncValue;
    }
    millivolts = constrain(
        millivolts,
        MIN_SMPS_VOLTAGE_MILLIVOLTS,
        MAX_SMPS_VOLTAGE_MILLIVOLTS
    );

    return millivolts;
}


// Update SMPS milliamps setting from the latest encoder reading
uint16_t updateMilliampsSMPS(uint16_t milliamps){
    currEncValue = currEncoder->getValue() * MILLIAMPS_PER_NOTCH;
    // Guard against overflow of uint
    // (can occur when millivolts = 0, or when millivolts = 0.01V and
    // curr_enc_val = -0.1V
    if((milliamps != 0 && abs(currEncValue) <= milliamps) || (currEncValue > 0)){
        milliamps += currEncValue;
    }
    milliamps = constrain(
        milliamps,
        MIN_SMPS_CURRENT_MILLIAMPS,
        MAX_SMPS_CURRENT_MILLIAMPS
    );

    return milliamps;
}

// Draw a value on screen, erasing old chars where necessary.
void draw_value(char valStr[], char oldValStr[], uint8_t len, uint8_t x,
                uint8_t y, uint8_t width, uint8_t height, uint8_t totalWidth)
{
    for(int i = 0; i < len; i++){
        // If a character has changed, erase it and redraw.
        if(valStr[i] != oldValStr[i])
        {
            // The x coordinate of the start of the character to change
            int charStartXCoord = x + i * totalWidth;
            // Blank out the char rectangle
            tft.fillRect(charStartXCoord, y, width, height, BLACK);

            tft.setCursor(charStartXCoord, y);
            tft.print(valStr[i]);
        }
    }

}


// Set Actual UI voltage for switched supply
void setUIVoltageActualSMPS(uint16_t millivolts) {
    tft.setTextColor(GREEN);
    tft.setTextSize(SMPS_VOLT_TEXT_SIZE);

    // Convert uint16t to str
    float volts = millivolts / 1000.0;
    char voltStr[SMPS_VOLT_NUM_CHARS];
    static char oldVoltStr[SMPS_VOLT_NUM_CHARS];
    dtostrf(volts, SMPS_VOLT_NUM_CHARS, 2, voltStr);

    draw_value(voltStr, oldVoltStr, SMPS_VOLT_NUM_CHARS, SMPS_VOLT_START_X,
               SMPS_VOLT_START_Y, SMPS_VOLT_CHAR_WIDTH, SMPS_VOLT_CHAR_HEIGHT,
               SMPS_VOLT_TOTAL_CHAR_WIDTH);

    strncpy(oldVoltStr, voltStr, SMPS_VOLT_NUM_CHARS);
}

// Set 'Set' UI voltage for switched supply
void setUIVoltageSetSMPS(uint16_t millivolts) {
    tft.setTextColor(GREEN);
    tft.setTextSize(SMPS_VSET_TEXT_SIZE);

    // Convert uint16t to str
    float volts = millivolts / 1000.0;
    char voltStr[SMPS_VSET_NUM_CHARS];
    static char oldVoltStr[SMPS_VSET_NUM_CHARS];
    dtostrf(volts, SMPS_VSET_NUM_CHARS, 2, voltStr);

    draw_value(voltStr, oldVoltStr, SMPS_VSET_NUM_CHARS, SMPS_VSET_START_X,
               SMPS_VSET_START_Y, SMPS_VSET_CHAR_WIDTH, SMPS_VSET_CHAR_HEIGHT,
               SMPS_VSET_TOTAL_CHAR_WIDTH);

    strncpy(oldVoltStr, voltStr, SMPS_VOLT_NUM_CHARS);
}

// Set UI voltage for switched supply
void setUIVoltageLinear(uint16_t millivoltsLinear) {
    tft.setTextColor(GREEN);
    tft.setTextSize(LINEAR_VOLT_TEXT_SIZE);

    // Convert uint16t to str
    float volts = millivoltsLinear / 1000.0;
    char voltStr[LINEAR_VOLT_NUM_CHARS];
    static char oldVoltStr[LINEAR_VOLT_NUM_CHARS];
    dtostrf(volts, LINEAR_VOLT_NUM_CHARS, 1, voltStr);

    draw_value(voltStr, oldVoltStr, LINEAR_VOLT_NUM_CHARS, LINEAR_VOLT_START_X,
               LINEAR_VOLT_START_Y, LINEAR_VOLT_CHAR_WIDTH,
               LINEAR_VOLT_CHAR_HEIGHT, LINEAR_VOLT_TOTAL_CHAR_WIDTH);

    strncpy(oldVoltStr, voltStr, LINEAR_VOLT_NUM_CHARS);
}

// Set UI current for switched supply
void setUICurrentSMPS(uint16_t milliamps) {
    tft.setTextColor(BLUE);
    tft.setTextSize(SMPS_CURR_TEXT_SIZE);

    // Convert uint16t to str
    float amps = milliamps / 1000.0;
    char currStr[SMPS_CURR_NUM_CHARS];
    static char oldCurrStr[SMPS_CURR_NUM_CHARS];
    dtostrf(amps, SMPS_CURR_NUM_CHARS, 2, currStr);

    draw_value(currStr, oldCurrStr, SMPS_CURR_NUM_CHARS, SMPS_CURR_START_X,
               SMPS_CURR_START_Y, SMPS_CURR_CHAR_WIDTH, SMPS_CURR_CHAR_HEIGHT,
               SMPS_CURR_TOTAL_CHAR_WIDTH);
    strncpy(oldCurrStr, currStr, SMPS_CURR_NUM_CHARS);
}


// Setup display with units etc
void setupUI() {
    // Setup SMPS Voltage unit
    int charStartXCoord = SMPS_VSET_START_X
                          + SMPS_VSET_NUM_CHARS * SMPS_VSET_TOTAL_CHAR_WIDTH;
    tft.setCursor(charStartXCoord, SMPS_VSET_START_Y);
    tft.setTextColor(GREEN);
    tft.setTextSize(SMPS_VSET_TEXT_SIZE);
    tft.print("V");

    charStartXCoord = SMPS_VOLT_START_X
                          + SMPS_VOLT_NUM_CHARS * SMPS_VOLT_TOTAL_CHAR_WIDTH;
    tft.setCursor(charStartXCoord, SMPS_VOLT_START_Y);
    tft.setTextSize(SMPS_VOLT_TEXT_SIZE);
    tft.print("V");

    // Setup Current unit
    charStartXCoord = SMPS_CURR_START_X
                      + SMPS_CURR_NUM_CHARS * SMPS_CURR_TOTAL_CHAR_WIDTH;
    tft.setCursor(charStartXCoord, SMPS_CURR_START_Y);
    tft.setTextColor(BLUE);
    tft.setTextSize(SMPS_CURR_TEXT_SIZE);
    tft.print("A");

    // Setup Linear Voltage unit
    charStartXCoord = LINEAR_VOLT_START_X
                      + LINEAR_VOLT_NUM_CHARS * LINEAR_VOLT_TOTAL_CHAR_WIDTH;
    tft.setCursor(charStartXCoord, LINEAR_VOLT_START_Y);
    tft.setTextColor(GREEN);
    tft.setTextSize(LINEAR_VOLT_TEXT_SIZE);
    tft.print("V");

    // Draw rectangles around SMPS and linear zones
    tft.drawRect(SMPS_ZONE_X, SMPS_ZONE_Y, ZONE_WIDTH, ZONE_HEIGHT, WHITE);
    tft.drawRect(LINEAR_ZONE_X, LINEAR_ZONE_Y, ZONE_WIDTH, ZONE_HEIGHT, WHITE);

    // Title the zones
    tft.setCursor(45, SMPS_ZONE_Y - 30);
    tft.setTextSize(3);
    tft.setTextColor(WHITE);
    tft.print("SMPS");

    tft.setCursor(187, LINEAR_ZONE_Y - 30);
    tft.print("LINEAR");

    // Use some humour words
    tft.setCursor(200, 130);
    tft.setTextColor(BLUE);
    tft.setTextSize(1);
    tft.print("There's no current ");
    tft.setCursor(200, 140);
    tft.print("limit lol");
}

// Sample ADC differential voltage from linear power supply
int16_t getLinearVoltageMillivolts(){
    float result = ads.readADC_Differential_0_1() * ADC_MULTIPLIER;
    return (int16_t) (result * ADC_0_1_DIVIDER_GAIN);
}

// Sample ADC differential voltage from transformer output
int16_t getTransformerVoltageMillivolts(){
    float result = ads.readADC_Differential_2_3() * ADC_MULTIPLIER;
    return (int16_t) (result * ADC_2_3_DIVIDER_GAIN);
}

// Show constant voltage symbol (hides CV)
void setUICV(bool enabled)
{

}


// Update encoder routines
void encoderISR() {
    voltEncoder->service();
    currEncoder->service();
}




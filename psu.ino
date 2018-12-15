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
#define SMPS_VOLT_START_X           10
#define SMPS_VOLT_START_Y           70
#define SMPS_VOLT_TEXT_SIZE         5
#define SMPS_VOLT_CHAR_WIDTH        (5 * SMPS_VOLT_TEXT_SIZE)
#define SMPS_VOLT_CHAR_HEIGHT       (7 * SMPS_VOLT_TEXT_SIZE)
#define SMPS_VOLT_CHAR_SPACER       2
#define SMPS_VOLT_TOTAL_CHAR_WIDTH  (SMPS_VOLT_CHAR_WIDTH + SMPS_VOLT_CHAR_SPACER)
#define SMPS_VOLT_NUM_CHARS         5

#define SMPS_CURR_START_X           10
#define SMPS_CURR_START_Y           120
#define SMPS_CURR_TEXT_SIZE         4
#define SMPS_CURR_CHAR_WIDTH        (5 * SMPS_CURR_TEXT_SIZE)
#define SMPS_CURR_CHAR_HEIGHT       (7 * SMPS_CURR_TEXT_SIZE)
#define SMPS_CURR_CHAR_SPACER       2
#define SMPS_CURR_TOTAL_CHAR_WIDTH  (SMPS_CURR_CHAR_WIDTH + SMPS_CURR_CHAR_SPACER)
#define SMPS_CURR_NUM_CHARS         5

/*
 * Rotary encoders
 * ===============
 *
 * Voltage encoder is on A8 to A10:
 * CLK  => A8
 * DT   => A9
 * BTN  => A10
 *
 * Current encoder is on 11 to 13:
 * CLK  => 11
 * DT   => 12
 * BTN  => 13
 */
#define ENC_VOLT_CLK A8
#define ENC_VOLT_DT  A9
#define ENC_VOLT_BTN A10

#define ENC_CURR_CLK 11
#define ENC_CURR_DT  12
#define ENC_CURR_BTN 13

// Define how much parameters should change for each click of the rotary encs
#define MILLIVOLTS_PER_NOTCH_SLOW   100
#define MILLIVOLTS_PER_NOTCH_FAST   1000
#define MILLIAMPS_PER_NOTCH_SLOW    100
#define MILLIAMPS_PER_NOTCH_FAST    1000

// Scale ADC input values according the attenuation by the voltage dividers
#define ADC_0_1_DIVIDER_GAIN        7.25F
#define ADC_2_3_DIVIDER_GAIN        10.72F
#define ADC_ADDRESS                 0x48
#define ADC_MULTIPLIER              0.1875F

#define MIN_SMPS_VOLTAGE_MILLIVOLTS 0
#define MAX_SMPS_VOLTAGE_MILLIVOLTS 40000

#define MIN_SMPS_CURRENT_MILLIAMPS  0
#define MAX_SMPS_CURRENT_MILLIAMPS  10000 // TBD

// When writing to the PSU, it is not possible to write to it again immediately
// after - it does not respond. It needs this delay to digest the message.
#define PSU_WRITE_DELAY_MS 100

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

ClickEncoder *voltEncoder;
ClickEncoder *currEncoder;

BuckPSU psu(Serial1);

Adafruit_ADS1115 ads(ADC_ADDRESS);

uint16_t millivolts = 0, prevMillivolts = -1;
uint16_t milliamps = 0, prevMilliamps = -1;

int16_t voltEncValue, currEncCalue;

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

    setUIVoltageSMPS(1000);
    setUICurrentSMPS(1000);
    psu.setVoltageMilliVolts(1000);
    psu.setCurrentMilliAmps(1000);
}

void loop()
{
    delay(200);
    Serial.print("Linear:  ");
    Serial.println(getLinearVoltageMillivolts());
    Serial.print("transformer:  ");
    Serial.println(getTransformerVoltageMillivolts());

    millivolts = updateMillivolts(millivolts);
    if (millivolts != prevMillivolts) {
        prevMillivolts = millivolts;

        setUIVoltageSMPS(millivolts);
        psu.setVoltageMilliVolts(millivolts);
        delay(PSU_WRITE_DELAY_MS);
    }


    ClickEncoder::Button currEncoder_btn = currEncoder->getButton();
    milliamps = updateMilliamps(milliamps);
    if (milliamps != prevMilliamps) {
        prevMilliamps = milliamps;

        setUICurrentSMPS(milliamps);
        psu.setCurrentMilliAmps(milliamps);
        delay(PSU_WRITE_DELAY_MS);
    }

}

// Update millivolts from the latest encoder reading
uint16_t updateMillivolts(uint16_t millivolts){
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


// Update milliamps from the latest encoder reading
uint16_t updateMilliamps(uint16_t milliamps){
    static bool currAdjustSlow = true;

    // If button is clicked, change speed from slow to fast or vice versa
    ClickEncoder::Button currEncoderBtn = currEncoder->getButton();
    if(currEncoderBtn == ClickEncoder::Clicked){
        currAdjustSlow = !currAdjustSlow;
    }
    if(currAdjustSlow){
        currEncCalue = currEncoder->getValue() * MILLIAMPS_PER_NOTCH_SLOW;
    }
    else{
        currEncCalue = currEncoder->getValue() * MILLIAMPS_PER_NOTCH_FAST;
    }
    // Guard against overflow of uint
    // (can occur when millivolts = 0, or when millivolts = 0.01V and
    // curr_enc_val = -0.1V
    if((milliamps != 0 && abs(currEncCalue) <= milliamps) || (currEncCalue > 0)){
        milliamps += currEncCalue;
    }
    milliamps = constrain(
        milliamps,
        MIN_SMPS_CURRENT_MILLIAMPS,
        MAX_SMPS_CURRENT_MILLIAMPS
    );

    return milliamps;
}

// Set UI voltage
void setUIVoltageSMPS(uint16_t millivolts) {
    tft.setCursor(SMPS_VOLT_START_X, SMPS_VOLT_START_Y);
    tft.setTextColor(GREEN);
    tft.setTextSize(SMPS_VOLT_TEXT_SIZE);

    // Convert uint16t to str
    float volts = millivolts / 1000.0;
    char voltStr[SMPS_VOLT_NUM_CHARS];
    static char oldVoltStr[SMPS_VOLT_NUM_CHARS];
    dtostrf(volts, SMPS_VOLT_NUM_CHARS, 2, voltStr);

    for(int i = 0; i < SMPS_VOLT_NUM_CHARS; i++){
        // If a character has changed, erase it and redraw.
        if(voltStr[i] != oldVoltStr[i])
        {
            // The x coordinate of the start of the character to change
            int charStartXCoord = SMPS_VOLT_START_X
                                  + i * SMPS_VOLT_TOTAL_CHAR_WIDTH;
            // Blank out the char rectangle
            tft.fillRect(
                charStartXCoord,        // x
                SMPS_VOLT_START_Y,      // y
                SMPS_VOLT_CHAR_WIDTH,   // width
                SMPS_VOLT_CHAR_HEIGHT,  // height
                BLACK                   // colour
            );
            tft.setCursor(charStartXCoord, SMPS_VOLT_START_Y);
            tft.print(voltStr[i]);
        }
    }
    strncpy(oldVoltStr, voltStr, 5);
}

// Set UI curent
void setUICurrentSMPS(uint16_t milliamps) {
    tft.setCursor(SMPS_CURR_START_X, SMPS_CURR_START_Y);
    tft.setTextColor(BLUE);
    tft.setTextSize(SMPS_CURR_TEXT_SIZE);

    // Convert uint16t to str
    float amps = milliamps / 1000.0;
    char currStr[SMPS_CURR_NUM_CHARS];
    static char oldCurrStr[SMPS_CURR_NUM_CHARS];
    dtostrf(amps, SMPS_CURR_NUM_CHARS, 2, currStr);

    for(int i = 0; i < SMPS_CURR_NUM_CHARS; i++){
        // If a character has changed, erase it and redraw.
        if(currStr[i] != oldCurrStr[i])
        {
            // The x coordinate of the start of the character to change
            int charStartXCoord = SMPS_CURR_START_X
                                  + i * SMPS_CURR_TOTAL_CHAR_WIDTH;
            // Blank out the char rectangle
            tft.fillRect(
                charStartXCoord,        // x
                SMPS_CURR_START_Y,      // y
                SMPS_CURR_CHAR_WIDTH,   // width
                SMPS_CURR_CHAR_HEIGHT,  // height
                BLACK                   // colour
            );
            tft.setCursor(charStartXCoord, SMPS_CURR_START_Y);
            tft.print(currStr[i]);
        }
    }
    strncpy(oldCurrStr, currStr, 5);
}

// Setup display with units etc
void setupUI() {
    // Setup Voltage unit
    int charStartXCoord = SMPS_VOLT_START_X
                          + SMPS_VOLT_NUM_CHARS * SMPS_VOLT_TOTAL_CHAR_WIDTH;
    tft.setCursor(charStartXCoord, SMPS_VOLT_START_Y);
    tft.setTextColor(GREEN);
    tft.setTextSize(SMPS_VOLT_TEXT_SIZE);
    tft.print("V");

    // Setup Current unit
    charStartXCoord = SMPS_CURR_START_X
                          + SMPS_CURR_NUM_CHARS * SMPS_CURR_TOTAL_CHAR_WIDTH;
    tft.setCursor(charStartXCoord, SMPS_CURR_START_Y);
    tft.setTextColor(BLUE);
    tft.setTextSize(SMPS_CURR_TEXT_SIZE);
    tft.print("A");
}

// Sample ADC differential voltage from linear power supply
int16_t getLinearVoltageMillivolts(){
    float result = ads.readADC_Differential_0_1() * ADC_MULTIPLIER;
    Serial.print("result: ");
    Serial.println(result);
    float result_scaled = result * ADC_0_1_DIVIDER_GAIN;
    Serial.print("result scaled: ");
    Serial.println(result_scaled);
    return (int16_t) (result_scaled);
}

// Sample ADC differential voltage from transformer output
int16_t getTransformerVoltageMillivolts(){
  float multiplier = 0.1875F;
    float voltage = ads.readADC_Differential_2_3() * multiplier;
    return (int16_t) (voltage * ADC_2_3_DIVIDER_GAIN);
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




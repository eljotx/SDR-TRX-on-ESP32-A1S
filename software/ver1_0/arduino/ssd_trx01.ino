//I would like to thank these colleagues for their undoubted contribution to the creation of this project:

//Ian A. Langle, NA5Y, for the inspiration regarding the ESP32A1S and Hilbert filters (see his YouTube 
//videos, such as https://www.youtube.com/watch?v=D0EUDRo4Mx0 )

//Phil Schatzmann https://github.com/pschatzmann/arduino-audio-tools for I2S support, filtering, 
//and likely for future inspiration

//Jasno Mildrum https://github.com/etherkit/si5351arduino for the library EtherKit for si5351

//John M. Wargo https://johnwargo.com/posts/2023/arduino...ple-cores/ for a description of how to use both cores 
//of the ESP32 processor

//Matias Hertel https://github.com/mathertel/RotaryEncod...r/examples for handling different encoder modes

//Bodmer https://github.com/Bodmer for the TFT library for esp processors

//Rui Santos https://randomnerdtutorials.com/esp32-tf...1-arduino/ for the practical configuration of the TFT library for ESP 
//and the ili9341 display

//Nigel Redmon https://www.earlevel.com/main/2021/09/02...ulator-v3/ for the online biquad filter calculator

//to the software developers at IowaHills (Windows only) https://web.archive.org/web/201711102019...hills.com/ for
// their Hilbert filter calculator (and more)


void (*resetFunc)(void) = 0;

//-----------------------------loading required libraries-------------------------------
#include <dummy.h>               //Error Supression Libary 
#include "freertos/FreeRTOS.h"   //ESP32 (R.T.O.S) Real Time Operating System
#include "freertos/task.h"       //RTOS Task handler (Controls System Tasks
#include "AudioTools.h"
#include "es8388.h"
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>  // Hardware-specific library
#include "Wire.h"
#include "si5351.h"
#include <Arduino.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>
#include "AudioTools/AudioLibs/AudioRealFFT.h"
//-----------------------------loading required graphic elements in graph directory-------------------------------
#include ".//graph//g_b.h"
#include ".//graph//g00f.h"
#include ".//graph//g00e.h"
#include ".//graph//g01f.h"
#include ".//graph//g01e.h"
#include ".//graph//g02f.h"
#include ".//graph//g02e.h"
#include ".//graph//g03f.h"
#include ".//graph//g03e.h"
#include ".//graph//g04f.h"
#include ".//graph//g04e.h"
#include ".//graph//g05f.h"
#include ".//graph//g05e.h"
#include ".//graph//g06f.h"
#include ".//graph//g06e.h"
#include ".//graph//g07f.h"
#include ".//graph//g07e.h"
#include ".//graph//g08f.h"
#include ".//graph//g08e.h"
#include ".//graph//g09f.h"
#include ".//graph//g09e.h"
#include ".//graph//g10f.h"
#include ".//graph//g10e.h"
#include ".//graph//g11f.h"
#include ".//graph//g11e.h"
#include ".//graph//g12f.h"
#include ".//graph//g12e.h"
#include ".//graph//g13f.h"
#include ".//graph//g13e.h"
#include ".//graph//g14f.h"
#include ".//graph//g14e.h"
#include ".//graph//g15f.h"
#include ".//graph//g15e.h"
#include ".//graph//save.h"
#include ".//graph//plus.h"
#include ".//graph//minus.h"
#include ".//graph//digit_green.h"
#include ".//graph//digit_gray.h"
#include ".//graph//rect_gray.h"
#include ".//graph//rect_green.h"
#include ".//graph//sq_gray.h"
#include ".//graph//sq_green.h"
#include ".//graph//walet0.h"
#include "hb_24_81.h"  //define hilbert transform 


TaskHandle_t Task0;


//-----------------------------define lp and bp filters-------------------------------
const float b_lpf24_36[] = { 0.13109923, 0.26219847, 0.13109923 }; //biquad lpf 24kHz sampling, 3600Hz f max
const float a_lpf24_36[] = { 1.0, -0.74774808, 0.27214502 };

const float b_lpf24_24[] = { 0.06745228, 0.13490457, 0.06745228 }; //biquad lpf 24kHz sampling, 2400Hz f max
const float a_lpf24_24[] = { 1.0, -1.14292982, 0.41273895 };

const float b_bpf24_8_6[] = { 0.01703090, 0.0, -0.01703090 };  //biquad bpf 24kHz sampling, 800Hz central freq
const float a_bpf24_8_6[] = { 1.0, -1.92297774, 0.96593821 };


//-----------------------------some audio definition and basic actions-------------------------------
uint16_t sample_rate = 24000;
uint16_t channels = 2;
uint16_t bits_per_sample = 16;  // or try with 24 or 32
//MultiOutput out;
//AudioRealFFT fft; //or AudioKissFFT
I2SStream i2s;
//StreamCopy copier(i2s, i2s); // copies sound into i2s
FilteredStream<int16_t, float> inFiltered(i2s, channels);  // Defiles the filter as BaseConverter
StreamCopy copier(i2s, inFiltered);                        // copies filtered audio to output
//StreamCopy copier(out, inFiltered); 


//-----------------------------Basic TFT and touch screen action-------------------------------
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
// This is the file name used to store the calibration data
// You can change this to create new calibration files.
// The SPIFFS file name must start with "/".
#define CALIBRATION_FILE "/TouchCalData1"
// Set REPEAT_CAL to true instead of false to run calibration
// again, otherwise it will only be done once.
// Repeat calibration if you change the screen rotation.
#define REPEAT_CAL false


//-----------------------------Some program and ports definitions-------------------------------
#define Freq_X 40  // Centre of key
#define Freq_Y 96
#define Freq_W 62  // Width and height
#define Freq_H 30
#define But_x 284
#define But_y 17
#define But_sp_y 40  // X and Y gap
#define PIN_IN1 36   //Rotary encoder pin definition
#define PIN_IN2 39
#define LOGO_HEIGHT 120
#define LOGO_WIDTH 240
#define RxTx_port 14  //port Rx/TX
#define Cwk_port 12   //port CW keying
#define Cwt_port 2  //CW ton 800Hz
#define Arw 34
#define But2s 4
#define RxTx_led_x 232
#define RxTx_led_y 13
#define Cws_x 224
#define Cws_y 124

//-----------------------------I2C hardware definitions-------------------------------
#define Bpf_addr 0x20  //Bpf addr PCF8574 32d
#define Lpf_addr 0x27  //Lpf addr PCF8574 swtich addr 39d
#define Att_addr 0x23  //RF addr PCF8574 attenuator addr 35d
#define Brg_addr 0x72  //Bridge addr addr PCF8574 114d


//-----------------------------Rotary encoder definition-------------------------------
//RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
//RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder *encoder = nullptr;
IRAM_ATTR void checkPosition() {
  encoder->tick();  // just call tick() to check the state.
}
// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
//RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);//
// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
//RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);


//-----------------------------Some grafics settings-------------------------------
#define BigFont &FreeMonoBold18pt7b
#define SmallFont &FreeSans9pt7b
// Numeric display box size and location
#define DISP_X 1
#define DISP_Y 10
#define DISP_W 238
#define DISP_H 50
#define DISP_TSIZE 3
#define DISP_TCOLOR TFT_CYAN
// Number length, buffer for storing it and character index
#define NUM_LEN 12
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;
uint8_t F1F2 = 0;

// We have a status line for messages
#define STATUS_X 120  // Centred on this
#define STATUS_Y 65

// Create 15 keys for the keypad
char keyLabel[15][5] = { "New", "Del", "Send", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "#" };
uint16_t keyColor[15] = { TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                          TFT_BLUE, TFT_BLUE, TFT_BLUE,
                          TFT_BLUE, TFT_BLUE, TFT_BLUE,
                          TFT_BLUE, TFT_BLUE, TFT_BLUE,
                          TFT_BLUE, TFT_BLUE, TFT_BLUE };
// Invoke the TFT_eSPI button class and create all the button objects
//TFT_eSPI_Button key[15];

//-----------------------------definitions of program constants and variables-------------------------------
String callsign;
String F1_str;
String F2_str;
uint16_t Bgr_color = TFT_BLACK;
uint16_t Freq_color = TFT_BLUE;
uint16_t But_color = TFT_BLUE;
uint16_t But_text = TFT_YELLOW;
uint16_t But_active = TFT_MAGENTA;
uint16_t But_normal = TFT_GREEN;
TFT_eSPI_Button Freq;
TFT_eSPI_Button CwSsb;
TFT_eSPI_Button UpDn;
TFT_eSPI_Button Rit;
TFT_eSPI_Button Opt1;
TFT_eSPI_Button Opt2;
TFT_eSPI_Button Opt3;
TFT_eSPI_Button Save;
TFT_eSPI_Button Plus;
TFT_eSPI_Button Minus;
TFT_eSPI_Button Call;
TFT_eSPI_Button Corre;
TFT_eSPI_Button Cwdelay;
bool CwSsb_state = true;
bool UpDn_state = true;
byte Att_value = 0x7E;  //0x7E;  //if UpDn_state = false ->0x7E, 0x7D, 0x7B, 0x77, 0x6F, 0x5F, 0x3F
//if UpDn_state = true ->0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF
bool Rit_state = false;
bool Menu_state = false;
bool Option_state = false;
bool Step_state = false;
bool RxTx_state = 0;  //receive/transmit
bool Setup_state = false;
bool isFirst = true;
bool isFilterChanged = true;


unsigned long long F1 = 350000000ULL;  //czestotliwosc podstawowa
unsigned long long F2 = 700000000ULL;  //czestotliwosc zapasowa
unsigned long long F03 = 350000000ULL;
unsigned long long F07 = 700000000ULL;  
unsigned long long F10 = 1010000000ULL; 
unsigned long long F14 = 1400000000ULL; 
unsigned long long F18 = 1806800000ULL; 
unsigned long long F21 = 2100000000ULL; 
unsigned long long F24 = 2489000000ULL; 
unsigned long long F28 = 2800000000ULL; 
unsigned long long Step[] = { 2000ULL, 10000ULL, 100000ULL, 1000000ULL, 10000000ULL, 100000000ULL };
String Step_txt[] = { " 20Hz", "100Hz", " 1kHz", "10kHz", " 100k", " 1MHz" };
signed long long Corr;  // definicja zmiennej korekcji Si5351
unsigned long long pll_freq = 60000000000ULL;
unsigned long long pll_tmp;
unsigned long long pll_div;
int Step_index = 1;
int Filter_index = 0;  //0 - 3600Hz LPF, 1 - 2400Hz LPF, 2 - 800Hz Q=6 BPF
//int Filter_index_old = 0; //used when switched to transmitt and return to receive
unsigned long delay1 = 250;
unsigned long delay2 = 30;
signed long TxRx_delay;  // = 500;  //delay when change from Tx to Rx in cw
unsigned long oldMs1 = 0;
unsigned long oldMs2 = 0;
unsigned long curMs1;
unsigned long curMs2;
int Band_index = 0;  // 0-3.5MH, 1-7zMHz, .....7-28MHz
int Band_index_old = Band_index;
String Band_txt[] = { "3.5", "7.0", "10.1", "14.0", "18.0", "21", "24.9", "28.0" };
int El_key_time[] = { 240, 200, 171, 150, 133, 120, 109, 100, 92, 86, 80, 75, 71, 67, 63, 60, 57, 55, 52, 50, 48 };  //21 speeds (dot time [ms]) from 5 to 25 groups/min
int El_key_speed = 9;                                                                                                //El_key_speed initial 14 (9+6) gr/min
int16_t x_of = 2;
int16_t y_of = 2;
//int m_digits[16][4]={{1,35,32,72},{16,29,27,74},{30,24,26,75},{47,20,20,75},
//  {62,17,17,76},{83,12,9,77},{99,12,6,77},{115,12,3,77},{128,12,6,78},{139,14,11,78},
//  {151,17,15,77},{163,20,20,77},{174,24,24,76},{186,29,27,75},{196,34,34,74},{207,34,31,74}};
int m_digit = 0;
int m_digit_old = m_digit;
int Arw_level = 0;
int Arw_level_old = Arw_level;

//S scale: S1-0.2uV,S2-0.4uV,S3-0.8uV,S4-1.6uV,S5-3.2uV,S6-6.3uV,S7-12.6uV,S8-25.1uV
//S9-50.2uV,S9+10-160uV,S9+20-500uV,S9+30-1.5mV,S9+40-5mV,S9+50-15mV,S9+60-50mV

// Define si5351 instance
Si5351 *si5351;
// Define Two I2C channels
TwoWire wire0(0);
TwoWire wire1(1);

String version = "ver 1.0";

//---------------------;---------------------------------------------------------------------

void setup(void) {
  // Use serial port
  Serial.begin(115200);

  //AudioLogger::instance().begin(Serial, AudioLogger::Error);

  //create a task that executes the Task0code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Task0code, "Task0", 100000, NULL, 1, &Task0, 0);

  pinMode(RxTx_port, OUTPUT);
  pinMode(Cwk_port, OUTPUT);
  pinMode(Cwt_port, OUTPUT);
  digitalWrite(Cwt_port, LOW); 
  analogWriteFrequency(Cwt_port, 800);  //setting cw ton frequency
  analogWrite(Cwt_port, 0);             //ton generator off

  EEPROM.begin(20);  //initialize eeprom with size 20 bytes
  //Reading data from EEPROM when system started
  callsign = "";
  for (int it = 0; it < 7; it++) {
    char callch = EEPROM.read(it);
    callsign = callsign + (char)callch;
    //callsign = "SP6FRE";
  }
  if (EEPROM.read(7) == 1) {
    Corr = -EEPROM.read(8) * 255 + EEPROM.read(9);
  } else {
    Corr = EEPROM.read(8) * 255 + EEPROM.read(9);
  }
  TxRx_delay = EEPROM.read(10) * 255 + EEPROM.read(11);

//defining encoder mode and interrupts
  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  //encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  //encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);


  //I2C pin definiton for channel 0 and channel 1
  wire0.setPins(33, 32);
  wire1.setPins(13, 15);  //SDA,SCL
  // si5351 connecting to second I2C channel
  si5351 = new Si5351(&wire1);
  si5351->init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351->set_correction(Corr, SI5351_PLL_INPUT_XO);
  set_Freq(0);  //start si5351 with frequency F1

//define filters chain
  //inFiltered.setFilter(0, new FIR<float>(hilbert_24_81));
  inFiltered.setFilter(0, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_36,a_lpf24_36), new FIR<float>(hilbert_24_81)}));
  //inFiltered.setFilter(0, new FilterChain<float, 2>({new BiQuadDF2<float>(b_bpf24_8_6,a_bpf24_8_6), new FIR<float>(hilbert_24_81)}));

  //inFiltered.setFilter(1, new FIR<float>(hilbert_24_81));
  inFiltered.setFilter(1, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_36,a_lpf24_36), new FIR<float>(coeffs_delay_81)}));
  //inFiltered.setFilter(1, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_24,a_lpf24_20), new FIR<float>(coeffs_delay_81)}));
  //inFiltered.setFilter(1, new FilterChain<float, 2>({new BiQuadDF2<float>(b_bpf24_8_6,a_bpf24_8_6), new FIR<float>(coeffs_delay_81)}));

  // Defining audio and codec
  es_dac_output_t output = (es_dac_output_t)(DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2);
  //es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;
  es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;

  es8388 codec;
  codec.begin(&wire0);
  codec.config(bits_per_sample, output, input, 90);

  // start I2S in
  //Serial.println("starting I2S...");
  auto config = i2s.defaultConfig(RXTX_MODE);
  config.sample_rate = sample_rate;
  config.bits_per_sample = bits_per_sample;
  config.channels = 2;
  config.i2s_format = I2S_STD_FORMAT;
  config.pin_ws = 25;
  config.pin_bck = 27;
  config.pin_data = 26;
  config.pin_data_rx = 35;
  config.pin_mck = 0;
  i2s.begin(config);

  //out.add(i2s);
  //out.add(fft);

  //Setup FFT output
  //auto tcfg = fft.defaultConfig();
  //tcfg.length = 128;
  //tcfg.callback = &fftResult;
  //fft.begin(tcfg);

  //Serial.println("I2S started...");

//Setting some values by I2S
  byte data_s;
  if (UpDn_state == true) {
    data_s = Att_value + 128;
  } else {
    data_s = Att_value;  // FE, FD, FB, F7
  }
  wire1.beginTransmission(Att_addr);  //Initial setting for RF Attn
  wire1.write(data_s);                // FE, FD, FB, F7
  wire1.endTransmission();
  wire1.beginTransmission(Bpf_addr);  //Initial setting for Bpf
  wire1.write(0x01);                  // 255-1=254 - P0 (3.5MHz) = HIGH, 255-2=253 - P1 (7MHz) = HIGH, .... 255-128=127 - P7 (28MHz) = HIGH
  wire1.endTransmission();
  wire1.beginTransmission(Lpf_addr);  //Initial setting for Lpf
  wire1.write(0x01);                  // 255-1=254 - P0 (4MHz) = HIGH, 255-2=253 - P1 (8MHz) = HIGH, 255-4=251 - P2 (16MHz), 255-8=247 - P3 (32MHz) = HIGH
  wire1.endTransmission();
  

  // Initialise the TFT screen
  tft.init();

  // Set the rotation before we calibrate
  tft.setRotation(1);

  // Calibrate the touch screen and retrieve the scaling factors
  touch_calibrate();
  //uint16_t calData[5] = { 335, 2900, 250, 3497, 1 };
  //tft.setTouch(calData);


  intro(); //Walet on screen
  // Clear the screen
  tft.fillScreen(TFT_BLACK);

  
//Drawing some elements on display
  tft.drawRect(0, 0, 245, 124, TFT_WHITE);   //ramka miernika
  tft.drawRect(0, 183, 245, 56, TFT_WHITE);  //ramka fft start, with, height
  //tft.drawLine(319,0, 319,239, TFT_WHITE);
  fftGraph(10); // lower area dedicated for future fft visualisation

  tft.setSwapBytes(true);

  tft.pushImage(x_of, y_of, 240, 120, g_b);  // Draw a bitmap
  delay(150);

  Meter(0, 0);

  // Draw keypad
  drawKeypad();

  tft.setTextDatum(ML_DATUM);  // Use middle left corner as text coord datum
  tft.setFreeFont(BigFont);
  tft.setTextColor(But_text);  // Set the font colour
  F1_str = Fstring(F1);
  tft.drawString(F1_str, 10, 140);
  tft.setTextColor(But_normal);
  tft.drawString(F1_str.substring(7, 8), 10 + 7 * 21, 140);
  tft.setTextColor(But_text);
  tft.setFreeFont(SmallFont);  
  tft.setTextDatum(ML_DATUM);
  tft.setFreeFont(SmallFont);  
  F2_str = Fstring(F2);
  tft.drawString("F2 " + F2_str, 10, 165);
  tft.setTextDatum(C_BASELINE);
  tft.drawString(callsign, 122, 116);
  tft.setTextColor(But_normal);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Step " + Step_txt[Step_index], 150, 165);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(But_text);
  tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
  tft.drawString(Band_txt[Band_index], But_x, But_y + 10);
  Band_Freq(Band_index);  //Setting band start freq
  RfFiltersSet();         //setting filters accordign to the band

  tft.drawString("CW", But_x, But_y + But_sp_y + 10);
  tft.drawString("F2=F1", But_x, But_y + 3 * But_sp_y + 10);
  tft.drawString("3.6k", But_x, But_y + 4 * But_sp_y + 10);
  tft.drawString("Mnu1", But_x, But_y + 5 * But_sp_y + 10);
  tft.fillCircle(RxTx_led_x, RxTx_led_y, 8, TFT_GREEN);          //green lamp whet RX
  tft.drawString(String(El_key_speed + 5) + "G", Cws_x, Cws_y);  //show actual CW speed
}


//------------------------------------------------------------------------------------------

void loop(void) {

  if (analogRead(But2s) > 3700) {  //detection Rx or Tx -> Rx mode

    curMs1 = millis();
    curMs2 = curMs1;
    if ((curMs1 - oldMs1) > delay1) {
      //int Arw_level = map(analogRead(Arw),0,2800,1,15);
      int Arw_read = analogRead(Arw);
      if (Arw_read < 50){Arw_level = 1;}
      else if (Arw_read >= 50 and Arw_read < 100){Arw_level = 2;}
      else if (Arw_read >= 100 and Arw_read < 150){Arw_level = 3;}
      else if (Arw_read >= 150 and Arw_read < 250){Arw_level = 4;}
      else if (Arw_read >= 250 and Arw_read < 400){Arw_level = 5;}
      else if (Arw_read >= 400 and Arw_read < 650){Arw_level = 6;}
      else if (Arw_read >= 650 and Arw_read < 900){Arw_level = 7;}
      else if (Arw_read >= 900 and Arw_read < 1200){Arw_level = 8;}
      else if (Arw_read >= 1200 and Arw_read < 1400){Arw_level = 9;}
      else if (Arw_read >= 1400 and Arw_read < 1600){Arw_level = 10;}
      else if (Arw_read >= 1600 and Arw_read < 1800){Arw_level = 11;}
      else if (Arw_read >= 1800 and Arw_read < 2000){Arw_level = 12;}
      else if (Arw_read >= 2000 and Arw_read < 2200){Arw_level = 13;}
      else if (Arw_read >= 2200 and Arw_read < 2400){Arw_level = 14;}
      else if (Arw_read >= 2400){Arw_level = 15;}
      //Serial.print(analogRead(Arw));
      //Serial.print("    ");
      //Serial.println(Arw_level);
      Meter(Arw_level_old, Arw_level);
      Arw_level_old = Arw_level;

      //fftGraph(random(0,15));
      fftGraph(0);
      if (Setup_state == false) {
        button_Check();  //checking if right panel button was pressed
      }
      oldMs1 = curMs1;
      //m_digit = 7 + random(-2,2);
      //if (m_digit != m_digit_old){
      //  Meter(m_digit_old,m_digit);
      //}
    }

    if ((curMs2 - oldMs2) > delay2) {
      static int pos = 0;
      encoder->tick();  // just call tick() to check the state.
      //encoder.tick();
      int newPos = encoder->getPosition();
      //int newPos = encoder.getPosition();
      if (pos != newPos) {
        int tmp = pos - newPos;
        if (Step_state == true) {
          uint16_t Step_index_old = Step_index;
          tft.setFreeFont(&FreeSans9pt7b);
          tft.setTextDatum(ML_DATUM);
          tft.setTextColor(Bgr_color);
          tft.drawString("Step " + Step_txt[Step_index], 150, 165);
          Step_index = Step_index + tmp;
          if (Step_index > 5) {
            Step_index = 0;
          }
          if (Step_index < 0) {
            Step_index = 5;
          }
          tft.setTextColor(But_active);
          tft.drawString("Step " + Step_txt[Step_index], 150, 165);
          Show_step(F1, Step_index_old, But_text);
          Show_step(F1, Step_index, But_active);
          tft.setFreeFont(SmallFont);
        } else {
          tft.setTextDatum(ML_DATUM);  // Use top left corner as text coord datum
          tft.setFreeFont(BigFont);
          String F1_str_old = Fstring(F1);
          set_Freq(pos - newPos);
          F1_str = Fstring(F1);
          Fstr_compare(F1_str_old, F1_str);
          //Band_Freq(Band_index);
          RfFiltersSet();  //setting bpf and lpf
        }
      }
      pos = newPos;
    }

  }
  else {    //detection Rx or Tx -> tx mode
    static int pos = 0;
    int key_pressed = map(analogRead(But2s), 0, 4096, 0, 5);
    //Serial.println(key_pressed);
    if (CwSsb_state == true) {   //Cw mode 
      si5351->output_enable(SI5351_CLK0, 0);
      si5351->output_enable(SI5351_CLK1, 0);
      si5351->output_enable(SI5351_CLK2, 0);
      oldMs1 = millis();
      digitalWrite(RxTx_port, HIGH);  //setting HIGH RxTx and Cwk signals
      digitalWrite(Cwk_port, HIGH);
      tft.fillCircle(RxTx_led_x, RxTx_led_y, 8, TFT_BLACK);  //
      tft.fillCircle(RxTx_led_x, RxTx_led_y, 8, TFT_RED);    //red lamp whet TX
      if (UpDn_state == 1) {
        si5351->set_freq_manual(F1 + 80000ULL, pll_tmp, SI5351_CLK2);
      } else {
        si5351->set_freq_manual(F1 - 80000ULL, pll_tmp, SI5351_CLK2);
      }
      while ((millis() - oldMs1) < TxRx_delay) {  //checking if still Tx mode
        static int pos = 0;
        key_pressed = map(analogRead(But2s), 0, 4096, 0, 5);
        if (key_pressed == 1) {
          oldMs1 = millis();             //zeroing cw count due to pressed key
          digitalWrite(Cwk_port, HIGH);  //
          analogWrite(Cwt_port, 127);
          si5351->output_enable(SI5351_CLK2, 1);
        } else if (key_pressed == 2) {
          oldMs1 = millis();
          oldMs2 = oldMs1;
          while (millis() - oldMs2 < El_key_time[El_key_speed]) {  //generating dot
            digitalWrite(Cwk_port, HIGH);                          //
            analogWrite(Cwt_port, 127);
            si5351->output_enable(SI5351_CLK2, 1);
          }
          oldMs1 = millis();
          oldMs2 = oldMs1;
          while (millis() - oldMs2 < El_key_time[El_key_speed]) {  //generating dot space
            digitalWrite(Cwk_port, LOW);                           //
            analogWrite(Cwt_port, 0);
            //si5351 -> output_enable(SI5351_CLK2, 0);
            encoder->tick();  // just call tick() to check the state, sppeed change during keing dot
            //encoder.tick();
            int newPos = encoder->getPosition();
            //int newPos = encoder.getPosition();
            if (pos != newPos) {
              tft.setTextDatum(BC_DATUM);
              tft.setFreeFont(&FreeSans9pt7b);
              tft.setTextColor(TFT_BLACK);
              tft.drawString(String(El_key_speed + 5) + "G", Cws_x, Cws_y);
              int tmp = pos - newPos;
              El_key_speed = tmp + El_key_speed;
              if (El_key_speed < 0) {
                El_key_speed = 0;
              }
              if (El_key_speed > 20) {
                El_key_speed = 20;
              }
              tft.setTextColor(But_text);
              tft.drawString(String(El_key_speed + 5) + "G", Cws_x, Cws_y);
              pos = newPos;
            }
          }
        } else if (key_pressed == 3) {
          oldMs1 = millis();
          oldMs2 = oldMs1;
          while (millis() - oldMs2 < 3 * El_key_time[El_key_speed]) {  //generating dash
            digitalWrite(Cwk_port, HIGH);                              //
            analogWrite(Cwt_port, 127);
            si5351->output_enable(SI5351_CLK2, 1);
          }
          oldMs1 = millis();
          oldMs2 = oldMs1;
          while (millis() - oldMs2 < El_key_time[El_key_speed]) {  //generating dot space
            digitalWrite(Cwk_port, LOW);                           //
            analogWrite(Cwt_port, 0);
            //si5351 -> output_enable(SI5351_CLK2, 0);
          }
        } else {
          digitalWrite(Cwk_port, LOW);
          analogWrite(Cwt_port, 0);
          //si5351 -> output_enable(SI5351_CLK2, 0);
        }
      }
      digitalWrite(RxTx_port, LOW);                          //ending TX in cw mode
      tft.fillCircle(RxTx_led_x, RxTx_led_y, 8, TFT_GREEN);  //green lamp whet RX
      si5351->output_enable(SI5351_CLK0, 1);
      si5351->output_enable(SI5351_CLK1, 1);
      si5351->output_enable(SI5351_CLK2, 0);
    } else { //ssb Tx mode
      tft.fillCircle(RxTx_led_x, RxTx_led_y, 8, TFT_RED);
      analogWrite(Cwt_port, 0);
      digitalWrite(RxTx_port, HIGH);  //setting HIGH RxTx and Cwk signals
      digitalWrite(Cwk_port, HIGH);
      int Filter_index_old;
      if (Filter_index != 1){ //if filter is different than 2400
        Filter_index_old = Filter_index; //save Rx filter index
        Filter_index =1;
        isFilterChanged = 1; //forcing setting Tx filter as 2400
      }
      while (key_pressed < 4) { //Tx mode when PTT pushed
        key_pressed = map(analogRead(But2s), 0, 4096, 0, 5);
        //measure output signals (power, swr) 
      }
      if (Filter_index_old != 1){ //return to previous filter state
        Filter_index = Filter_index_old;
        isFilterChanged = 1; //forcing setting Rx filter as previous if different than 2400
      }
      digitalWrite(RxTx_port, LOW);  //setting HIGH RxTx and Cwk signals
      digitalWrite(Cwk_port, LOW);
      tft.fillCircle(RxTx_led_x, RxTx_led_y, 8, TFT_GREEN);  //green lamp whet RX
    }
  }
}


void fftGraph(int strenght){ //under development
  int color;
  if (strenght == 0){
    color = TFT_BLUE;
  }
  if (strenght == 1){
    color = TFT_GREEN;
  }
  if (strenght == 2){
    color = TFT_CYAN;
  }
  if (strenght == 3){
    color = TFT_RED;
  }
  if (strenght == 4){
    color = TFT_MAGENTA;
  }
  if (strenght == 5){
    color = TFT_YELLOW;
  }
  if (strenght == 6){
    color = TFT_WHITE;
  }
  if (strenght == 7){
    color = TFT_ORANGE;
  }
  if (strenght == 8){
    color = TFT_PINK;
  }
  if (strenght == 9){
    color = TFT_BROWN;
  }
  if (strenght == 10){
    color = TFT_GOLD;
  }
  if (strenght == 11){
    color = TFT_SILVER;
  }
  if (strenght == 12){
    color = TFT_BLACK;
  }
  if (strenght == 13){
    color = TFT_NAVY;
  }
  if (strenght == 14){
    color = TFT_PURPLE;
  }
  if (strenght == 15){
    color = TFT_OLIVE;
  }
  for (int ind = 185; ind < 237;ind ++){
      tft.drawFastHLine(2,ind,241,color);
  } 

}


void fftResult(AudioFFTBase &fft) {  //under development
  AudioFFTResult weights[64];
  fft.resultArray(weights);
    //auto result = fft.result(); // Gets the dominant frequency peak    
    // To extract all individual frequency bins in array:
    //float* weights = fft.resultArray();
    //int totalBins = fft.size() / 2; 
    //for (int bin = 0; bin < 8; bin++){
    //	Serial.print("Freq = ");
    //	Serial.print(weights[bin].frequency);
    //	Serial.print(" Magnitude = ");
    //	Serial.print(weights[bin].magnitude);
    //}
    //Serial.println(" ");
    //Serial.print("Dominant Freq: ");
    //Serial.print(result.frequency);
    //Serial.print(" Hz | Magnitude: ");
    //Serial.println(result.magnitude);
}  


//core 0 definition
void Task0code(void * pvParameters) {
  //Serial.print("Task0 running on core ");
  //Serial.println(xPortGetCoreID());
  for (;;) {
    //Serial.println("Core 0 processing");
    //delay((int)random(100, 1000));
    copier.copy();
    if (isFilterChanged == true){
      if (Filter_index == 0){
        inFiltered.setFilter(0, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_36,a_lpf24_36), new FIR<float>(hilbert_24_81)}));
        inFiltered.setFilter(1, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_36,a_lpf24_36), new FIR<float>(coeffs_delay_81)}));
      }
      else if (Filter_index == 1){
        inFiltered.setFilter(0, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_24,a_lpf24_24), new FIR<float>(hilbert_24_81)}));
        inFiltered.setFilter(1, new FilterChain<float, 2>({new BiQuadDF2<float>(b_lpf24_24,a_lpf24_24), new FIR<float>(coeffs_delay_81)}));
      }
      else if (Filter_index == 2){
        inFiltered.setFilter(0, new FilterChain<float, 2>({new BiQuadDF2<float>(b_bpf24_8_6,a_bpf24_8_6), new FIR<float>(hilbert_24_81)}));
        inFiltered.setFilter(1, new FilterChain<float, 2>({new BiQuadDF2<float>(b_bpf24_8_6,a_bpf24_8_6), new FIR<float>(coeffs_delay_81)}));
      }
      isFilterChanged = false;
    }
    delay(1);
  }
}


//Walet on the start screen
void intro(){
tft.fillScreen(TFT_BLUE);
delay(100);
tft.pushImage(89,5,142,200,walet0);
delay(500);
tft.setTextDatum(ML_DATUM);  // Use middle left corner as text coord datum
tft.setFreeFont(BigFont);
tft.setTextColor(TFT_YELLOW);  // Set the font colour
tft.drawString(version, 89, 220);
delay(3000);
}

//Setup definitions: call sign, Si5351 correction, cw delay etc.
void Settings() {
  Setup_state = true;
  si5351->output_enable(SI5351_CLK0, 0);
  si5351->output_enable(SI5351_CLK1, 0);
  si5351->output_enable(SI5351_CLK2, 1);
  F1 = 1000000000ULL;
  set_Freq(0);
  si5351->set_freq_manual(2 * F1, pll_tmp, SI5351_CLK2);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(C_BASELINE);
  tft.setFreeFont(SmallFont);
  bool saved = false;

  Save.initButton(&tft, 275, 220, 85, 30, TFT_WHITE, But_color, But_text, "SAVE", 1);  //&tft, x, y, w, h, outline, fill, text
  Save.drawButton();

  tft.setFreeFont(BigFont);
  Plus.initButton(&tft, 297, 175, 37, 37, TFT_WHITE, But_color, But_text, "+", 1);  //&tft, x, y, w, h, outline, fill, text
  Plus.drawButton();

  Minus.initButton(&tft, 250, 175, 37, 37, TFT_WHITE, But_color, But_text, "-", 1);  //&tft, x, y, w, h, outline, fill, text
  Minus.drawButton();
  tft.setFreeFont(SmallFont);
  Corre.initButton(&tft, 43, 20, 85, 37, TFT_WHITE, TFT_MAGENTA, But_text, "Correct", 1);  //&tft, x, y, w, h, outline, fill, text
  Corre.drawButton();
  tft.fillRoundRect(95, 3, 100, 37, 5, TFT_DARKGREY);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString(String(Corr), 145, 28);
  int setItem = 1;
  int setItemold = setItem;

  Call.initButton(&tft, 43, 62, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "Call Sgn", 1);  //&tft, x, y, w, h, outline, fill, text
  Call.drawButton();
  tft.fillRoundRect(95, 44, 35, 37, 5, TFT_DARKGREY);
  tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_DARKGREY);
  tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_DARKGREY);
  tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_DARKGREY);
  tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_DARKGREY);
  tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
  int digit_pos = 5;
  tft.setFreeFont(BigFont);
  int cslength = 7;
  char csarray[cslength];
  callsign.toCharArray(csarray, cslength);
  for (int tmp = 0; tmp < cslength; tmp++) {
    tft.drawChar(csarray[tmp], 104 + tmp * 37, 70);
  }

  tft.setFreeFont(SmallFont);
  Cwdelay.initButton(&tft, 43, 104, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "CW delay", 1);  //&tft, x, y, w, h, outline, fill, text
  Cwdelay.drawButton();
  tft.fillRoundRect(95, 85, 100, 37, 5, TFT_DARKGREY);
  tft.drawString(String(TxRx_delay), 143, 110);

  int count_step = 1;
  int count_tens = 0;
  uint16_t t_x = 0, t_y = 0;
  while (saved == false) {
    bool pressed = tft.getTouch(&t_x, &t_y);
    if (pressed) {
      if (t_y < 40 && t_x < 65) {
        Corre.initButton(&tft, 43, 20, 85, 37, TFT_WHITE, TFT_MAGENTA, But_text, "Correct", 1);  //&tft, x, y, w, h, outline, fill, text
        Corre.drawButton();
        Call.initButton(&tft, 43, 62, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "Call Sgn", 1);  //&tft, x, y, w, h, outline, fill, text
        Call.drawButton();
        Cwdelay.initButton(&tft, 43, 104, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "CW delay", 1);  //&tft, x, y, w, h, outline, fill, text
        Cwdelay.drawButton();
        setItem = 1;
      }
      if (t_y > 45 && t_y < 80 && t_x < 65) {
        Corre.initButton(&tft, 43, 20, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "Correct", 1);  //&tft, x, y, w, h, outline, fill, text
        Corre.drawButton();
        Call.initButton(&tft, 43, 62, 85, 37, TFT_WHITE, TFT_MAGENTA, But_text, "Call Sgn", 1);  //&tft, x, y, w, h, outline, fill, text
        Call.drawButton();
        Cwdelay.initButton(&tft, 43, 104, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "CW delay", 1);  //&tft, x, y, w, h, outline, fill, text
        Cwdelay.drawButton();
        setItem = 2;
        while (setItem == 2) {
          char letter;
          int asciivalue;
          tft.setFreeFont(BigFont);
          bool pressed = tft.getTouch(&t_x, &t_y);
          if (pressed) {
            if ((t_x > 95 && t_x < 130) && (t_y > 44 && t_y < 81)) {
              tft.fillRoundRect(95, 44, 35, 37, 5, TFT_MAGENTA);
              tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
              digit_pos = 0;
              letter = csarray[digit_pos];
              asciivalue = (char)letter;
              //tft.setFreeFont(BigFont);
              //int cslength = 7;
              //char csarray[cslength];
              //callsign.toCharArray(csarray,cslength);
              for (int tmp = 0; tmp < cslength; tmp++) {
                tft.drawChar(csarray[tmp], 104 + tmp * 37, 70);
              }
            }
            if ((t_x > 132 && t_x < 167) && (t_y > 44 && t_y < 81)) {
              tft.fillRoundRect(95, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_MAGENTA);
              tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
              digit_pos = 1;
              letter = csarray[digit_pos];
              asciivalue = (char)letter;
              //tft.setFreeFont(BigFont);
              //int cslength = 7;
              //char csarray[cslength];
              //callsign.toCharArray(csarray,cslength);
              for (int tmp = 0; tmp < cslength; tmp++) {
                tft.drawChar(csarray[tmp], 104 + tmp * 37, 70);
              }
            }
            if ((t_x > 169 && t_x < 204) && (t_y > 44 && t_y < 81)) {
              tft.fillRoundRect(95, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_MAGENTA);
              tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
              digit_pos = 2;
              letter = csarray[digit_pos];
              asciivalue = (char)letter;
              //tft.setFreeFont(BigFont);
              //int cslength = 7;
              //char csarray[cslength];
              //callsign.toCharArray(csarray,cslength);
              for (int tmp = 0; tmp < cslength; tmp++) {
                tft.drawChar(csarray[tmp], 104 + tmp * 37, 70);
              }
            }
            if ((t_x > 206 && t_x < 241) && (t_y > 44 && t_y < 81)) {
              tft.fillRoundRect(95, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_MAGENTA);
              tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
              digit_pos = 3;
              letter = csarray[digit_pos];
              asciivalue = (char)letter;
              //tft.setFreeFont(BigFont);
              //int cslength = 7;
              //char csarray[cslength];
              //callsign.toCharArray(csarray,cslength);
              for (int tmp = 0; tmp < cslength; tmp++) {
                tft.drawChar(csarray[tmp], 104 + tmp * 37, 70);
              }
            }
            if ((t_x > 243 && t_x < 278) && (t_y > 44 && t_y < 81)) {
              tft.fillRoundRect(95, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_MAGENTA);
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
              digit_pos = 4;
              letter = csarray[digit_pos];
              asciivalue = (char)letter;
              //tft.setFreeFont(BigFont);
              //int cslength = 7;
              //char csarray[cslength];
              //callsign.toCharArray(csarray,cslength);
              for (int tmp = 0; tmp < cslength; tmp++) {
                tft.drawChar(csarray[tmp], 104 + tmp * 37, 70);
              }
            }
            if ((t_x > 280 && t_x < 315) && (t_y > 44 && t_y < 81)) {
              tft.fillRoundRect(95, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 37, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 74, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 111, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 148, 44, 35, 37, 5, TFT_DARKGREY);
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_MAGENTA);
              digit_pos = 5;
              letter = csarray[digit_pos];
              asciivalue = (char)letter;
              //tft.setFreeFont(BigFont);
              //int cslength = 7;
              //char csarray[cslength];
              //callsign.toCharArray(csarray,cslength);
              for (int tmp = 0; tmp < cslength; tmp++) {
                tft.drawChar((char)csarray[tmp], 104 + tmp * 37, 70);
              }
            }
            if ((t_x > 230 && t_x < 270) && (t_y > 155 && t_y < 195)) {  //minus
              tft.setTextColor(TFT_MAGENTA);
              tft.drawChar(csarray[digit_pos], 104 + digit_pos * 37, 70);
              asciivalue = asciivalue - 1;
              if (asciivalue < 65 && asciivalue > 57) {
                asciivalue = 57;
              }
              if (asciivalue < 48) {
                asciivalue = 32;
              }
              tft.setTextColor(TFT_YELLOW);
              csarray[digit_pos] = asciivalue;
              tft.drawChar(csarray[digit_pos], 104 + digit_pos * 37, 70);
            }
            if ((t_x > 275 && t_x < 320) && (t_y > 155 && t_y < 195)) {  //plus
              tft.setTextColor(TFT_MAGENTA);
              tft.drawChar(csarray[digit_pos], 104 + digit_pos * 37, 70);
              asciivalue = asciivalue + 1;
              if (asciivalue > 32 && asciivalue < 48) {
                asciivalue = 48;
              }
              if (asciivalue > 57 && asciivalue < 65) {
                asciivalue = 65;
              }
              if (asciivalue > 90) {
                asciivalue = 90;
              }
              tft.setTextColor(TFT_YELLOW);
              csarray[digit_pos] = (char)asciivalue;
              tft.drawChar(csarray[digit_pos], 104 + digit_pos * 37, 70);
            }
            if ((t_y < 40 && t_x < 65) || (t_y > 87 && t_y < 120 && t_x < 65)) {
              tft.fillRoundRect(95 + 185, 44, 35, 37, 5, TFT_DARKGREY);
              tft.setTextColor(TFT_YELLOW);
              tft.drawChar(csarray[digit_pos], 104 + digit_pos * 37, 70);
              setItem = 0;
              tft.setFreeFont(SmallFont);
            }
            delay(250);
          }
        }
      }
      if (t_y > 87 && t_y < 120 && t_x < 65) {
        Corre.initButton(&tft, 43, 20, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "Correct", 1);  //&tft, x, y, w, h, outline, fill, text
        Corre.drawButton();
        Call.initButton(&tft, 43, 62, 85, 37, TFT_WHITE, TFT_DARKGREY, But_text, "Call Sgn", 1);  //&tft, x, y, w, h, outline, fill, text
        Call.drawButton();
        Cwdelay.initButton(&tft, 43, 104, 85, 37, TFT_WHITE, TFT_MAGENTA, But_text, "CW delay", 1);  //&tft, x, y, w, h, outline, fill, text
        Cwdelay.drawButton();
        setItem = 3;
      }
      if ((t_x > 230 && t_x < 270) && (t_y > 155 && t_y < 195)) {  //minus
        if (setItem == 1) {
          tft.setTextColor(TFT_DARKGREY);
          tft.drawString(String(Corr), 145, 28);
          Corr = Corr - count_step;
          si5351->set_correction(Corr, SI5351_PLL_INPUT_XO);
          set_Freq(0);
          si5351->set_freq_manual(2 * F1, pll_tmp, SI5351_CLK2);
          tft.setTextColor(TFT_YELLOW);
          tft.drawString(String(Corr), 145, 28);
        }
        if (setItem == 3) {
          tft.setTextColor(TFT_DARKGREY);
          tft.drawString(String(TxRx_delay), 143, 110);
          TxRx_delay = TxRx_delay - count_step;
          if (TxRx_delay < 10) {
            TxRx_delay = 10;
          }
          tft.setTextColor(TFT_YELLOW);
          tft.drawString(String(TxRx_delay), 143, 110);
        }
        count_tens = count_tens + 1;
        if (count_tens >= 10) {
          count_step = count_step * 10;
          if (count_step > 1000) {
            count_step = 1000;
          }
          count_tens = 0;
        }
      }
      if ((t_x > 275 && t_x < 320) && (t_y > 155 && t_y < 195)) {
        if (setItem == 1) {
          tft.setTextColor(TFT_DARKGREY);
          tft.drawString(String(Corr), 145, 28);
          Corr = Corr + count_step;
          si5351->set_correction(Corr, SI5351_PLL_INPUT_XO);
          set_Freq(0);
          si5351->set_freq_manual(2 * F1, pll_tmp, SI5351_CLK2);
          tft.setTextColor(TFT_YELLOW);
          tft.drawString(String(Corr), 145, 28);
        }
        if (setItem == 3) {
          tft.setTextColor(TFT_DARKGREY);
          tft.drawString(String(TxRx_delay), 143, 110);
          TxRx_delay = TxRx_delay + count_step;
          if (TxRx_delay > 2000) {
            TxRx_delay = 2000;
          }
          tft.setTextColor(TFT_YELLOW);
          tft.drawString(String(TxRx_delay), 143, 110);
        }
        count_tens = count_tens + 1;
        if (count_tens >= 10) {
          count_step = count_step * 10;
          if (count_step > 1000) {
            count_step = 1000;
          }
          count_tens = 0;
        }
      }
      if ((t_x > 233 && t_x < 318) && (t_y > 205 && t_y < 235)) {
        //writing data to EEPROM
        callsign = csarray[0] + csarray[1] + csarray[2] + csarray[3] + csarray[4] + csarray[5];
        for (int tmp = 0; tmp < 6; tmp++) {
          int tmp1 = (char)csarray[tmp];
          EEPROM.write(tmp, tmp1);
        }
        if (Corr < 0) {
          EEPROM.write(7, 1);
          Corr = -Corr;
        } else {
          EEPROM.write(7, 0);
        }
        EEPROM.write(8, floor(Corr / 255));
        EEPROM.write(9, Corr - 255 * floor(Corr / 255));
        EEPROM.write(10, floor(TxRx_delay / 255));
        EEPROM.write(11, TxRx_delay - 255 * floor(TxRx_delay / 255));
        EEPROM.commit();
        Save.initButton(&tft, 275, 220, 85, 30, TFT_WHITE, TFT_MAGENTA, But_text, "SAVE", 1);  //&tft, x, y, w, h, outline, fill, text
        Save.drawButton();
        delay(1000);
        resetFunc();
      }
    } else {
      count_step = 1;
      count_tens = 0;
    }
    delay(250);
  }
}

//movement of the meter needle -> 16 positions
void Meter(int old, int cur) {
  switch (old) {
    case 0:
      tft.pushImage(x_of + 1, y_of + 35, 32, 72, g00e);
      break;
    case 1:
      tft.pushImage(x_of + 16, y_of + 29, 27, 74, g01e);
      break;
    case 2:
      tft.pushImage(x_of + 30, y_of + 24, 26, 75, g02e);
      break;
    case 3:
      tft.pushImage(x_of + 47, y_of + 20, 20, 75, g03e);
      break;
    case 4:
      tft.pushImage(x_of + 62, y_of + 17, 17, 76, g04e);
      break;
    case 5:
      tft.pushImage(x_of + 83, y_of + 12, 9, 78, g05e);
      break;
    case 6:
      tft.pushImage(x_of + 99, y_of + 12, 6, 77, g06e);
      break;
    case 7:
      tft.pushImage(x_of + 115, y_of + 12, 3, 77, g07e);
      break;
    case 8:
      tft.pushImage(x_of + 128, y_of + 12, 6, 78, g08e);
      break;
    case 9:
      tft.pushImage(x_of + 139, y_of + 14, 11, 78, g09e);
      break;
    case 10:
      tft.pushImage(x_of + 151, y_of + 17, 15, 77, g10e);
      break;
    case 11:
      tft.pushImage(x_of + 163, y_of + 20, 20, 77, g11e);
      break;
    case 12:
      tft.pushImage(x_of + 174, y_of + 24, 24, 76, g12e);
      break;
    case 13:
      tft.pushImage(x_of + 186, y_of + 29, 27, 75, g13e);
      break;
    case 14:
      tft.pushImage(x_of + 196, y_of + 34, 31, 74, g14e);
      break;
    case 15:
      tft.pushImage(x_of + 207, y_of + 34, 31, 74, g15e);
      break;
  }
  switch (cur) {
    case 0:
      tft.pushImage(x_of + 1, y_of + 35, 32, 72, g00f);
      break;
    case 1:
      tft.pushImage(x_of + 16, y_of + 29, 27, 74, g01f);
      break;
    case 2:
      tft.pushImage(x_of + 30, y_of + 24, 26, 75, g02f);
      break;
    case 3:
      tft.pushImage(x_of + 47, y_of + 20, 20, 75, g03f);
      break;
    case 4:
      tft.pushImage(x_of + 62, y_of + 17, 17, 76, g04f);
      break;
    case 5:
      tft.pushImage(x_of + 83, y_of + 12, 9, 78, g05f);
      break;
    case 6:
      tft.pushImage(x_of + 99, y_of + 12, 6, 77, g06f);
      break;
    case 7:
      tft.pushImage(x_of + 115, y_of + 12, 3, 77, g07f);
      break;
    case 8:
      tft.pushImage(x_of + 128, y_of + 12, 6, 78, g08f);
      break;
    case 9:
      tft.pushImage(x_of + 139, y_of + 14, 11, 78, g09f);
      break;
    case 10:
      tft.pushImage(x_of + 151, y_of + 17, 15, 77, g10f);
      break;
    case 11:
      tft.pushImage(x_of + 163, y_of + 20, 20, 77, g11f);
      break;
    case 12:
      tft.pushImage(x_of + 174, y_of + 24, 24, 76, g12f);
      break;
    case 13:
      tft.pushImage(x_of + 186, y_of + 29, 27, 75, g13f);
      break;
    case 14:
      tft.pushImage(x_of + 196, y_of + 34, 31, 74, g14f);
      break;
    case 15:
      tft.pushImage(x_of + 207, y_of + 34, 31, 74, g15f);
      break;
  }
  m_digit_old = m_digit;
}


//saving the frequency for each band
void Band_Freq(int index) {
  tft.setFreeFont(SmallFont);
  String tmp = Fstring(F1);
  if (index == 0) {
    //F1 = 350000000UL;
    if (isFirst == false){
      F28 = F1;
    }
    else{
      isFirst = false;
    }
    F1 = F03;
  } else if (index == 1) {
    //F1 = 700000000UL;
    F03 = F1;
    F1 = F07;
  } else if (index == 2) {
    F07 = F1;
    F1 = F10;
    //F1 = 1010000000UL;
  } else if (index == 3) {
    //F1 = 1400000000UL;
    F10 = F1;
    F1 = F14;
  } else if (index == 4) {
    //F1 = 1806800000UL;
    F14 = F1;
    F1 = F18;
  } else if (index == 5) {
    //F1 = 2100000000UL;
    F18 = F1;
    F1 = F21;
  } else if (index == 6) {
    //F1 = 2489000000UL;
    F21 = F1;
    F1 = F24;
  } else if (index == 7) {
    //F1 = 2800000000UL;
    F24 = F1;
    F1 = F28;
  }
  if (F1 >= 1000000000UL) {
    if (UpDn_state == false) {  //change UpDn button from DN to UP
      tft.setTextColor(But_color);
      tft.setTextDatum(BC_DATUM);
      tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
      tft.setTextColor(But_text);
      tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
      UpDn_state = true;
    }
  } else {
    if (UpDn_state = true) {  //change UpDn button from UP to DN
      tft.setTextDatum(BC_DATUM);
      tft.setTextColor(But_color);
      tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
      tft.setTextColor(But_text);
      tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
      UpDn_state = false;
    }
  }
  F1_str = Fstring(F1);
  Fstr_compare(tmp, F1_str);
  set_Freq(0);
  //wire1.endTransmission();
}

//setting the RF BPF and LPF filters on the hb and hc boards
void RfFiltersSet() {
  wire1.beginTransmission(Lpf_addr);  //Initial setting for Lpf
  if (F1 < 400000000UL) {
    wire1.write(0x01);  // 0xFE 1 - P0 (4MHz) = HIGH, 2 - P1 (8MHz) = HIGH, 4 - P2 (16MHz), 8 - P3 (32MHz) = HIG
  } else if (F1 >= 400000000UL && F1 < 800000000UL) {
    wire1.write(0x02);  // 0xFD 1 - P0 (4MHz) = HIGH, 2 - P1 (8MHz) = HIGH, 4 - P2 (16MHz), 8 - P3 (32MHz) = HIGH
  } else if (F1 >= 800000000UL && F1 < 1600000000UL) {
    wire1.write(0x04);  // 0xFB 1 - P0 (4MHz) = HIGH, 2 - P1 (8MHz) = HIGH, 4 - P2 (16MHz), 8 - P3 (32MHz) = HIGH
  } else if (F1 >= 1600000000UL) {
    wire1.write(0x08);  // 0xF7 1 - P0 (4MHz) = HIGH, 2 - P1 (8MHz) = HIGH, 4 - P2 (16MHz), 8 - P3 (32MHz) = HIGH
  }
  wire1.endTransmission();

  wire1.beginTransmission(Bpf_addr);  //Initial setting for Bpf
  if (F1 < 400000000UL) {
    wire1.write(0x01);  //0xFE  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 0;
    //Serial.println("Pasmo 3.5");
  } else if (F1 >= 400000000UL && F1 < 800000000UL) {
    wire1.write(0x02);  //0xFD  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 1;
    //Serial.println("Pasmo 7.0");
  } else if (F1 >= 800000000UL && F1 < 1200000000UL) {
    wire1.write(0x04);  //0xFB  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 2;
    //Serial.println("Pasmo 10.1");
  } else if (F1 >= 1200000000UL && F1 < 1600000000UL) {
    wire1.write(0x08);  //0xF7  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 3;
  } else if (F1 >= 1600000000UL && F1 < 2000000000UL) {
    wire1.write(0x10);  //0xEF  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 4;
  } else if (F1 >= 2000000000UL && F1 < 2200000000UL) {
    wire1.write(0x20);  //0xDF  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 5;
  } else if (F1 >= 2200000000UL && F1 < 2600000000UL) {
    wire1.write(0x40);  //0xBF  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 6;
  } else if (F1 >= 2600000000UL) {
    wire1.write(0x80);  //0x7F  1 - P0 (3.5MHz) = HIGH, 2 - P1 (7MHz) = HIGH, .... 128 - P7 (28MHz) = HIGH
    Band_index = 7;
  }
  wire1.endTransmission();
  if (Band_index_old != Band_index) {
    tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(But_color);
    tft.drawString(Band_txt[Band_index_old], But_x, But_y + 10);
    if (Band_index_old == 1) {
      tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
    }
    if (Band_index_old == 7) {
      tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
    }
    tft.setTextColor(But_text);
    tft.drawString(Band_txt[Band_index], But_x, But_y + 10);
    if (Band_index_old == 1) {
      tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
      UpDn_state = true;
    }
    if (Band_index_old == 7) {
      tft.setTextDatum(BC_DATUM);
      tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
      UpDn_state = false;
    }
    Band_index_old = Band_index;
  }
}

//visualization and modification of the selected tuning step
void Show_step(long long F, uint16_t Step_index, uint16_t color) {
  tft.setTextDatum(ML_DATUM);  // Use top left corner as text coord datum
  tft.setFreeFont(BigFont);
  tft.setTextColor(color);
  String Fstr = Fstring(F);
  String F_char;
  int pos;
  if (Step_index >= 0 | Step_index <= 1) {
    F_char = Fstr.substring(8 - Step_index, 9 - Step_index);
    pos = 10 + (8 - Step_index) * 21;
  }
  if (Step_index >= 2 && Step_index <= 4) {
    F_char = Fstr.substring(7 - Step_index, 8 - Step_index);
    pos = 10 + (7 - Step_index) * 21;
  }
  if (Step_index == 5) {
    F_char = Fstr.substring(6 - Step_index, 7 - Step_index);
    pos = 31;
  }
  tft.setTextColor(color);
  tft.drawString(F_char, pos, 140);
}

//changing the frequency F0 and F1
void Fstr_compare(String Str_old, String Str_new) {
  for (uint16_t i = 0; i < 9; i++) {
    if (Str_new != Str_old) {
      tft.setTextDatum(ML_DATUM);  // Use top left corner as text coord datum
      tft.setFreeFont(BigFont);
      tft.setTextColor(Bgr_color);
      tft.drawString(Str_old.substring(i, i + 1), 10 + i * 21, 140);
      if ((i == 1 && Step_index == 5) | ((i == 3 && Step_index == 4) | (i == 4 && Step_index == 3)) | (i == 5 && Step_index == 2) | (i == 7 && Step_index == 1) | (i == 8 && Step_index == 0)) {
        tft.setTextColor(But_normal);
      } else {
        tft.setTextColor(But_text);
      }
      tft.drawString(Str_new.substring(i, i + 1), 10 + i * 21, 140);
    }
  }
}

//changing the frequency value to a string
String Fstring(long long F) {
  String Fstr = String(F);
  if (Fstr.length() == 9) {
    return (" " + Fstr.substring(0, 1) + "." + Fstr.substring(1, 4) + "." + Fstr.substring(4, 7));
  } else {
    return (Fstr.substring(0, 2) + "." + Fstr.substring(2, 5) + "." + Fstr.substring(5, 8));
  }
}

//checking for and responding to an on-screen button press
void button_Check() {
  uint16_t t_x = 0, t_y = 0;  // To store the touch coordinates
  bool pressed = tft.getTouch(&t_x, &t_y);
  if (pressed) {
    if ((t_y >= 130 && t_y <= 170) && (t_x >= 5 && t_x <= 160)) {
      if (Step_state == false) {     //Checking if swap F1/F2 or change frequency step
        tft.setTextDatum(ML_DATUM);  // Use top left corner as text coord datum
        tft.setFreeFont(BigFont);
        tft.setTextColor(TFT_BLACK);  // Set the font colour
        F1_str = Fstring(F1);
        String tmp = F1_str;
        tft.drawString(tmp, 10, 140);
        tft.setTextColor(But_text);
        F2_str = Fstring(F2);
        tmp = F2_str;
        tft.drawString(tmp, 10, 140);
        Show_step(F2, Step_index, But_normal);
        tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(Bgr_color);
        F2_str = Fstring(F2);
        tmp = "F2 " + F2_str;
        tft.drawString(tmp, 10, 165);
        tft.setTextColor(But_text);
        F1_str = Fstring(F1);
        tmp = "F2 " + F1_str;
        tft.drawString(tmp, 10, 165);
        long long tmp3 = F1;
        F1 = F2;
        F2 = tmp3;
        tft.setTextDatum(BC_DATUM);
        if (F1 >= 1000000000UL) {
          if (UpDn_state == false) {  //change UpDn button from DN to UP
            tft.setTextColor(But_color);
            tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
            tft.setTextColor(But_text);
            tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
            UpDn_state = true;
          }
        } else {
          if (UpDn_state = true) {  //change UpDn button from UP to DN
            tft.setTextColor(But_color);
            tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
            tft.setTextColor(But_text);
            tft.setTextDatum(BC_DATUM);
            tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
            UpDn_state = false;
          }
        }
        set_Freq(0);
        //wire1.endTransmission();
        RfFiltersSet();
      } else {
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(But_normal);
        tft.drawString("Step " + Step_txt[Step_index], 150, 165);
        Show_step(F1, Step_index, But_normal);
        Step_state = false;
      }
    }
    if ((t_y >= 0 && t_y <= 25) && (t_x >= 250 && t_x <= 300)) {
      //if ((t_y >= (But_y + 20) && t_y <= (But_y + But_sp_y + 15))  && (t_x >= 250 && t_x <= 300)) {
      //tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      //tft.setTextDatum(BC_DATUM);
      //tft.setTextColor(But_color);
      //tft.drawString(Band_txt[Band_index], But_x, But_y + 10);
      Band_index = Band_index + 1;
      if (Band_index > 7) {
        Band_index = 0;
      }
      //tft.setTextColor(But_text);
      //tft.drawString(Band_txt[Band_index], But_x, But_y + 10);
      Band_Freq(Band_index);  //change and setting start band freq
      RfFiltersSet();
    }
    //if ((t_y >= 0 && t_y <= 25)  && (t_x >= 250 && t_x <= 300)) {
    if ((t_y >= (But_y + 20) && t_y <= (But_y + But_sp_y + 15)) && (t_x >= 250 && t_x <= 300)) {
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(BC_DATUM);
      if (CwSsb_state == true) {
        tft.setTextColor(But_color);
        tft.drawString("CW", But_x, But_y + But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("SSB", But_x, But_y + But_sp_y + 10);
        CwSsb_state = false;
      } else {
        tft.setTextColor(But_color);
        tft.drawString("SSB", But_x, But_y + But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("CW", But_x, But_y + But_sp_y + 10);
        CwSsb_state = true;
      }
    }
    //if ((t_y >= (But_y + 20) && t_y <= (But_y + But_sp_y + 15))  && (t_x >= 250 && t_x <= 300)) {
    if ((t_y >= (But_y + But_sp_y + 20) && t_y <= (But_y + 2 * But_sp_y + 15)) && (t_x >= 250 && t_x <= 319)) {
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(BC_DATUM);
      if (UpDn_state == true) {
        tft.setTextColor(But_color);
        tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
        UpDn_state = false;
      } else {

        tft.setTextColor(But_color);
        tft.drawString("DN", But_x, But_y + 2 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("UP", But_x, But_y + 2 * But_sp_y + 10);
        UpDn_state = true;
      }
      set_Freq(0);
      //wire1.endTransmission();
    }
    //if ((t_y >= (But_y + But_sp_y + 20) && t_y <= (But_y + 2 * But_sp_y + 15))  && (t_x >= 250 && t_x <= 319)) {
    if ((t_y >= (But_y + 2 * But_sp_y + 20) && t_y <= (But_y + 3 * But_sp_y + 15)) && (t_x >= 250 && t_x <= 319)) {
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(BC_DATUM);
      tft.setTextColor(TFT_MAGENTA);
      tft.drawString("F2=F1", But_x, But_y + 3 * But_sp_y + 10);
      tft.setTextDatum(ML_DATUM);
      tft.setTextColor(TFT_MAGENTA);
      F2_str = Fstring(F2);
      String tmp = "F2 " + F2_str;
      tft.drawString(tmp, 10, 165);
      delay(250);
      tft.setTextColor(Bgr_color);
      //F2_str = Fstring(F2);
      //tmp = "F2 " + F2_str;
      tft.drawString(tmp, 10, 165);
      F2 = F1;
      F2_str = Fstring(F2);
      tmp = "F2 " + F2_str;
      tft.setTextColor(But_text);
      tft.drawString(tmp, 10, 165);
      tft.setTextDatum(BC_DATUM);
      tft.drawString("F2=F1", But_x, But_y + 3 * But_sp_y + 10);
    }
    //if ((t_y >= (But_y + 2 *But_sp_y + 20) && t_y <= (But_y + 3 * But_sp_y + 15))  && (t_x >= 250 && t_x <= 319)) {
    if ((t_y >= (But_y + 3 * But_sp_y + 20) && t_y <= (But_y + 4 * But_sp_y + 15)) && (t_x >= 250 && t_x <= 319)) {
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(BC_DATUM);
      if (Filter_index == 0) {
        tft.setTextColor(But_color);
        tft.drawString("3.6k", But_x, But_y + 4 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("2.4k", But_x, But_y + 4 * But_sp_y + 10);
        //inFiltered.setFilter(0, new FilterChain<float, 2>({ new BiQuadDF1<float>(b_lpf24_24, a_lpf24_24), new FIR<float>(hilbert_24_81) }));
        //inFiltered.setFilter(1, new FilterChain<float, 2>({ new BiQuadDF1<float>(b_lpf24_24, a_lpf24_24), new FIR<float>(coeffs_delay_81) }));
        isFilterChanged = true;
        Filter_index = 1;
      } else if (Filter_index == 1) {
        tft.setTextColor(But_color);
        tft.drawString("2.4k", But_x, But_y + 4 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("0.8k", But_x, But_y + 4 * But_sp_y + 10);
        //inFiltered.setFilter(0, new FilterChain<float, 2>({ new BiQuadDF1<float>(b_bpf24_8_6, a_bpf24_8_6), new FIR<float>(hilbert_24_81) }));
        //inFiltered.setFilter(1, new FilterChain<float, 2>({ new BiQuadDF1<float>(b_bpf24_8_6, a_bpf24_8_6), new FIR<float>(coeffs_delay_81) }));
        isFilterChanged = true;
        Filter_index = 2;
      } else if (Filter_index == 2) {
        tft.setTextColor(But_color);
        tft.drawString("0.8k", But_x, But_y + 4 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("3.6k", But_x, But_y + 4 * But_sp_y + 10);
        //inFiltered.setFilter(0, new FilterChain<float, 2>({ new BiQuadDF1<float>(b_lpf24_36, a_lpf24_36), new FIR<float>(hilbert_24_81) }));
        //inFiltered.setFilter(1, new FilterChain<float, 2>({ new BiQuadDF1<float>(b_lpf24_36, a_lpf24_36), new FIR<float>(coeffs_delay_81) }));
        //inFiltered.setFilter(0, new FIR<float>(hilbert_24_81));
        //inFiltered.setFilter(1, new FIR<float>(hilbert_24_81));
        isFilterChanged = true;
        Filter_index = 0;
      }
    }
    if ((t_y >= (But_y + 4 * But_sp_y + 20) && t_y <= (But_y + 5 * But_sp_y + 15)) && (t_x >= 250 && t_x <= 319)) {
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(BC_DATUM);
      if (Menu_state == false) {
        tft.setTextColor(But_color);
        tft.drawString("Mnu1", But_x, But_y + 5 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("Mnu2", But_x, But_y + 5 * But_sp_y + 10);
        Settings();
        Menu_state = true;
      } else {
        tft.setTextColor(But_color);
        tft.drawString("Mnu2", But_x, But_y + 5 * But_sp_y + 10);
        tft.setTextColor(But_text);
        tft.drawString("Mnu1", But_x, But_y + 5 * But_sp_y + 10);
        Menu_state = false;
      }
    }
    if ((t_y >= 130 && t_y <= 190) && (t_x >= 180 && t_x <= 240)) {
      Show_step(F1, Step_index, But_text);
      Step_state = true;
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(ML_DATUM);
      tft.setTextColor(TFT_BLACK);
      tft.drawString("Step " + Step_txt[Step_index], 150, 165);
      tft.setTextColor(But_active);
      Step_index = Step_index + 1;
      if (Step_index > 5) {
        Step_index = 0;
      }
      tft.drawString("Step " + Step_txt[Step_index], 150, 165);
      Show_step(F1, Step_index, But_active);
      tft.setFreeFont(SmallFont);  // Choose a nice font that fits box
      tft.setTextDatum(ML_DATUM);
      tft.setTextColor(But_active);
    }
  }
}

//Si5351 generator frequency setting according to rotary encoder value 
void set_Freq(int dir) {
  //if dir = 0 just set freq without changing
  if (dir == 1 or dir == -1) {
    F1 = F1 + dir * Step[Step_index];
  }
  if (F1 > 5000000000ULL) {  //resttriction for upper freq
    F1 = 5000000000ULL;
  }
  if (F1 < 330000000ULL) {  //restriction for lower freq -> don't change it
    F1 = 330000000ULL;
  }
  long long pll_lim;
  if (F1 < 680000000ULL) {
    pll_lim = 40000000000ULL;
  } else {
    pll_lim = 70000000000ULL;
  }
  //calculate si5351 division parameters
  //pll_div = int(40000000000ULL / F1);
  pll_div = int(pll_lim / F1);
  if (pll_div & 1 == 1) {
    pll_div++;
  }
  pll_tmp = pll_div * F1;
  //while (pll_tmp < 40000000000ULL) {
  while (pll_tmp < pll_lim) {
    pll_div++;
    if (pll_div & 1 == 1) {
      pll_div++;
    }
    pll_tmp = pll_div * F1;
  }
  //Serial.print("PLL_A = ");
  //Serial.print(pll_lim);
  //Serial.print("   DIV = ");
  //Serial.print(pll_div);
  //Serial.print("   F1 = ");
  //Serial.println(F1);

  //setting si5351 freq clock0 and clock1 with 90 degree shiftment
  si5351->set_freq_manual(F1 * 2, pll_tmp, SI5351_CLK0);
  byte data_s;
  if (UpDn_state == true) {
    data_s = Att_value + 128;  // FE, FD, FB, F7
    //data_s = 0;  // FE, FD, FB, F7
  } else {
    data_s = Att_value;  // FE, FD, FB, F7
    //data_s = 255;
  }
  wire1.beginTransmission(Att_addr);  //Initial setting for RF Attn
  wire1.write(data_s);                // FE, FD, FB, F7
  wire1.endTransmission();
  /*
  //section for 90 degree created just in Si5351
  si5351->set_freq_manual(F1, pll_tmp, SI5351_CLK1);
  if (UpDn_state == true) {
    si5351->set_phase(SI5351_CLK0, 0);
    si5351->set_phase(SI5351_CLK1, pll_div);
    //si5351->pll_reset(SI5351_PLLB);
  } else {
    si5351->set_phase(SI5351_CLK0, pll_div);
    si5351->set_phase(SI5351_CLK1, 0);
    //si5351->pll_reset(SI5351_PLLA);
  }
  si5351->pll_reset(SI5351_PLLA);
  */
  //Wire1.beginTransmission(0x60); // Si5351 I2C address (0x60)
  //Wire1.write(177);              // Register 177
  //Wire1.write(128);             // Reset command for PLLA and PLLB (0xA0)
  //Wire1.endTransmission();
}

//drawing touchscreen keys
void drawKeypad() {

  CwSsb.initButton(&tft, But_x, But_y, 60, 30, TFT_WHITE, But_color, But_text, "", 1);  //&tft, x, y, w, h, outline, fill, text
  CwSsb.drawButton();

  UpDn.initButton(&tft, But_x, But_y + But_sp_y, 60, 30, TFT_WHITE, But_color, But_text, "", 1);  //&tft, x, y, w, h, outline, fill, text
  UpDn.drawButton();

  Rit.initButton(&tft, But_x, But_y + 2 * But_sp_y, 60, 30, TFT_WHITE, But_color, But_text, "", 1);  //&tft, x, y, w, h, outline, fill, text
  Rit.drawButton();

  Opt1.initButton(&tft, But_x, But_y + 3 * But_sp_y, 60, 30, TFT_WHITE, But_color, But_text, "", 1);  //&tft, x, y, w, h, outline, fill, text
  Opt1.drawButton();

  Opt2.initButton(&tft, But_x, But_y + 4 * But_sp_y, 60, 30, TFT_WHITE, But_color, But_text, "", 1);  //&tft, x, y, w, h, outline, fill, text
  Opt2.drawButton();

  Opt3.initButton(&tft, But_x, But_y + 5 * But_sp_y, 60, 30, TFT_WHITE, But_color, But_text, "", 1);  //&tft, x, y, w, h, outline, fill, text
  Opt3.drawButton();

  //Opt3.initButton(&tft,But_x,But_y+5*But_sp_y,60,30,TFT_WHITE,But_color,But_text,"",1); //&tft, x, y, w, h, outline, fill, text
  //Opt3.drawButton();
}

//------------------------------------------TFT calibration screen------------------------------------------------

//*

void touch_calibrate() {
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL) {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    } else {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

//*/

//------------------------------------------------------------------------------------------

// Print something in the mini status bar
void status(const char *msg) {
  tft.setTextPadding(240);
  //tft.setCursor(STATUS_X, STATUS_Y);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(0);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(1);
  tft.drawString(msg, STATUS_X, STATUS_Y);
}

//------------------------------------------------------------------------------------------

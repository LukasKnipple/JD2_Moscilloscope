/*

Written by: Lukas Knipple
Class: Junior Design II @ Oregon State Unversity
Date: Spring Term, 2025

This program contains all the code for operating Team 2's Moscilloscope Project (mobile oscilloscope). It is divided into its three functions:
1. ADC - Sample the input voltage from the moscilloscope's two channels using the Teensy v4.1's two ADCs
2. UI - Interface with the two rotary encoders and four pushbuttons to allow the user to interact with the moscilloscope
2. Display - Take all of the inputs and output them onto a 320x240 TFT display 

Reading through this program, you will find all the code grouped by these three categories^^. This is both for ease of debugging/reading as well 
as because this program was originally written in three separate parts (for simplicity) before being combined.

Additionally, this code was written for final integration with the moscilloscope hardware, as well as 'dummy' mode where fake data is generated, and
user UI input is prompted from the serial monitor (terminal). By assigning the "dummy" constant to TRUE or FALSE, the entire program can be swapped
between these modes

*/




#include <Arduino.h>

/* DISPLAY Libraries */
#include <ILI9341_T4.h>         // For the display being used (ILI9341, TFT Display)
#include <tgx.h>                // An open source 2d & 3D graphics library for Teensy (draw rectangles, lines, etc.)
#include "font_tgx_Arial.h"     // Fonts available through TGX
#include <string.h>
#include <math.h>

/* UI Libraries */
#include <Encoder.h>            // For reading two-bit quadrature off a rotary encoder
#include <bounce2.h>            // For accurately reading button presses, avoiding the 'bounce' (electrical noise) that occurs when initially pressed

/* ADC Libraries */
#include <ADC.h>                // ADC library for Teensy microcontroller. Allows greater utilization of the ADCs


#define runUI true
#define debugging true

// 'Fake' mode or real mode
#define DUMMY  false

/* DISPLAY Constants & Variables*/
// Assign human-readable names to some common 16-bit color values:
#define BLACK   tgx::RGB565_Black
#define GREEN   tgx::RGB565_Green
#define WHITE   tgx::RGB565_White
#define BLUE    tgx::RGB565_Blue
#define RED     tgx::RGB565_Red
#define YELLOW  tgx::RGB565_Yellow
#define CYAN    tgx::RGB565_CYAN

/* Define all pin assigments for the Teensy v4.1 */
#define PIN_SCK     13      // mandatory
#define PIN_MISO    12      // mandatory
#define PIN_MOSI    11      // mandatory
#define PIN_DC      10      // mandatory

#define PIN_CS      36      // optional (but recommended), can be any pin.  
#define PIN_RESET   37      // optional (but recommended), can be any pin. 
#define PIN_BACKLIGHT 255   // optional, set this only if the screen LED pin is connected directly to the Teensy.
#define PIN_TOUCH_IRQ 255   // optional. set this only if the touchscreen is connected on the same SPI bus
#define PIN_TOUCH_CS  255   // optional. set this only if the touchscreen is connected on the same spi bus

#define SPI_SPEED       30000000

#define LX 320
#define LY 240

double triggerVoltage = 1.23;
double VScale = 10;
double HScale = 8E-6; //HScale is the time/unit as seen on the oscilloscope, with 1 unit = 10 pixels (i.e. time/10 indices of the raw data array)
double CH1_P2P = 1;
double CH1_T = 0.00081;
double CH2_P2P = 1;
double CH2_T = 0.00081;

int menuOptionsX1;
int menuOptionsX2;
int menuOptionsY1;
int menuOptionsY2;
int menuOptionsYSpace;

tgx::iVec2 sig1Points[LX];
tgx::iVec2 sig2Points[LX];

double sig1Data[LX];
double sig2Data[LX];


String string_TrigVolt;

char* string_HScale = new char[30];
char* string_VScale = new char[30];
char* string_CH1_P2P = new char[30];
char* string_CH1_T = new char[30];
char* string_CH2_P2P = new char[30];
char* string_CH2_T = new char[30];

// Constants defining some font & color characteristics of different display components
#define CH1_COLOR          GREEN
#define CH2_COLOR          BLUE
#define MEAS_FONT          font_tgx_Arial_8
#define TRIG_VOLT_FONT     font_tgx_Arial_8
#define SCALE_FONT         font_tgx_Arial_8
#define MENU_FONT          font_tgx_Arial_8
#define CHANGE_VALUE_FONT  font_tgx_Arial_12
#define MENU_COLOR         WHITE

// the screen driver object
ILI9341_T4::ILI9341Driver tft(PIN_CS, PIN_DC, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_RESET, PIN_TOUCH_CS, PIN_TOUCH_IRQ);

uint16_t fb[LX*LY] = {0}; // our memory framebuffer. The screen has size 240 x 320 with color in 16 bits - RGB565 format.  

DMAMEM uint16_t fb_internal[LX*LY];  // the 'internal' frame buffer

ILI9341_T4::DiffBuffStatic<6000> diff1; // a first diff buffer with 4K memory (statically allocated)
ILI9341_T4::DiffBuffStatic<6000> diff2; // and a second one 

tgx::Image<tgx::RGB565> oScopeImage(fb, 320, 240);

/**/




/*UI Constants & Variables */

#define ENC_1A 20
#define ENC_1B 19
#define ENC_2A 16
#define ENC_2B 15
#define BUTTON_1_PIN    18   // Encoder 1's button
#define BUTTON_2_PIN    17   // Encoder 2's button
#define BUTTON_3_PIN    21   // Freestanding button 1 
#define BUTTON_4_PIN    14   // Freestanding button 2
#define ENC_Sensitivity 4 // Number of 'clicks' needed to register as an increment

#define MAX_TRIGGER           5.0 // Maximum trigger voltage value
#define TRIGGER_Sensitivity   0.01 // The trigger voltage increment for every UI reigstered rotary increment
#define MAX_HSCALE            102E-6
#define MIN_HSCALE            8E-6
#define HSCALE_Sensitivity    1E-6
#define MAX_VSCALE            20
#define VSCALE_Sensitivity    0.1


// Variables for storing user interface menu navigation data
int menuSelecting;
int menuSelected;
bool showMenu;

// Variables for storing user interface "channels menu" navigation data, and channel display data
int chDataSelecting;
int chDataSelected;
bool showWave1 = true;
bool showWave2 = true;
bool showMeas1;
bool showMeas2;

// Variables for storing PEC12R rotary encoder (and button) data
Encoder encoder1(ENC_1A, ENC_1B);
Encoder encoder2(ENC_2A, ENC_2B);
Bounce button1 = Bounce();
Bounce button2 = Bounce();
Bounce button3 = Bounce();
Bounce button4 = Bounce();
int enc1Old = 0;
int enc1New = 0;
int enc2Old = 0;
int enc2New = 0;
bool button1State = 0;
bool button2State = 0;
bool button3State = 0;
bool button4State = 0;
/**/




/* ADC Variables & Constants */

#define NUM_SAMPLES 4000 // for 4000 samples, the usable HScale range is 8.153 us to 101.9 us
#define CH1_PIN 41
#define CH2_PIN 23

#define ADC_RESOLUTION    10      // Resolution in bits
#define ADC_OVERSAMPLING  0      // 
#define SAMPLING_INTERVAL 1      // microseconds

#define upperVoltage 5
#define lowerVoltage -5



double sampleDt = 1.2265-6; // 0.81533 micro second = time to sample one data point from an ADC (time different (Dt) between each index in the sample array)

double HScaleMax = ((NUM_SAMPLES*1.0)*sampleDt)/32; // 153.31 microsecond @ NUM_SAMPLES = 4000
double HScaleMin = (sampleDt*10.0); // 12.265 microseconds @ NUM_SAMPLES = 4000

int rawData1[NUM_SAMPLES];
int rawData2[NUM_SAMPLES];
double voltageData1[NUM_SAMPLES];
double voltageData2[NUM_SAMPLES];
double offset1 = 0;
double offset2 = 0;


int sig1TrigIndex = 0;
int sig2TrigIndex = 0;

int startTime;
int elapsedTime;
elapsedMicros currentTime;

ADC *adc = new ADC();

/**/


/*
Name: bound
Description: Bounds a provided double-type to a provided range
Returns: Nothing (edits global variable)
Parameters: reference to a double (&val), double lower (lower bound), and double upper (upper bound)
*/
void bound(double &val, double lower, double upper){
  if(val < lower){
    val = lower;
  }
  if(val > upper){
    val = upper;
  }
}

/*
Name: bound
Description: Bounds a provided double-type to a provided range
Returns: Nothing (edits global variable)
Parameters: reference to a int (&val), int lower (lower bound), and int upper (upper bound)
*/
void bound(int &val, int lower, int upper){
  if(val < lower){
    val = lower;
  }
  if(val > upper){
    val = upper;
  }
}

#if runUI
// ---------------------
/* BEGIN UI Functions */
// ---------------------




/*
Name: checkButton1
Description: Returns the boolean state of button 1
Returns: bool "button1State"
Parameters: None
*/
// Returns the boolean state of button 1
bool checkButton1(){
  #if DUMMY
    int input;

    Serial.println("ENTER BUTTON 1");
    while(Serial.available() == 0){

    }
    input = Serial.parseInt();

    button1State = input;
    // Serial.print("Button1 State: ");
    Serial.println(button1State);
  #endif
  #if !DUMMY
    button1.update();
    button1State = button1.fell();
  #endif

  return button1State;
}

/*
Name: checkButton2
Description: Returns the boolean state of button 2
Returns: bool "button2State"
Parameters: None
*/
// Returns the boolean state of button 2
bool checkButton2(){
  #if DUMMY
    int input;

    Serial.println("ENTER BUTTON 2");
    while(Serial.available() == 0){

    }
    input = Serial.parseInt();

    button2State = input;
    // Serial.print("Button2 State: ");
    Serial.println(button2State);
  #endif
  #if !DUMMY
    button2.update();
    button2State = button2.fell();
  #endif
  
  return button2State;
}

/*
Name: checkButton3
Description: Returns the boolean state of button 3
Returns: bool "button3State"
Parameters: None
*/
// Returns the boolean state of button 3
bool checkButton3(){
  #if DUMMY
    int input;

    Serial.println("ENTER BUTTON 3");
    while(Serial.available() == 0){

    }
    input = Serial.parseInt();

    button3State = input;
    // Serial.print("Button3 State: ");
    Serial.println(button3State);
  #endif
  #if !DUMMY
    button3.update();
    button3State = button3.fell();
  #endif
  
  return button3State;
}

/*
Name: checkButton4
Description: Returns the boolean state of button 4
Returns: bool "button4State"
Parameters: None
*/
// Returns the boolean state of button 4
bool checkButton4(){
  #if DUMMY
    int input;

    Serial.println("ENTER BUTTON 4");
    while(Serial.available() == 0){

    }
    input = Serial.parseInt();

    button4State = input;
    // Serial.print("Button4 State: ");
    Serial.println(button4State);
  #endif
  #if !DUMMY
    button4.update();
    button4State = button4.fell();
  #endif
  
  return button4State;
}

/*
Name: readEncoder1Change
Description: Returns the number of rotary clicks registered by encoder 1. Actual clicks are divided out by ENC_SENSITIVITY to prevent
misreads from small variations in the encoder rotation.
Returns: int "difference"
Parameters: None
*/
int readEncoder1Change(){
  int difference;

  #if DUMMY
    int input;

    Serial.println("ENTER ENCODER 1");
    while(Serial.available() == 0){

    }
    input = Serial.parseInt();

    difference = input;
    // Serial.print("Encoder1 Change: ");
    Serial.println(difference);
  #endif

  #if !DUMMY
  enc1New = encoder1.read();

  difference = enc1New - enc1Old;
  #endif

  if(abs(difference) >= ENC_Sensitivity){
    enc1Old = enc1New;
    return difference / ENC_Sensitivity;
  }else{
    return 0;
  }
}

/*
Name: readEncoder2Change
Description: Returns the number of rotary clicks registered by encoder 2. Actual clicks are divided out by ENC_SENSITIVITY to prevent
misreads from small variations in the encoder rotation.
Returns: int "difference"
Parameters: None
*/
int readEncoder2Change(){
  int difference;

  #if DUMMY
    int input;

    Serial.println("ENTER ENCODER 2");
    while(Serial.available() == 0){

    }
    input = Serial.parseInt();

    difference = input;
    Serial.println(difference);
  #endif

  #if !DUMMY
  enc2New = encoder2.read();

  difference = enc2New - enc2Old;
  #endif

  if(abs(difference) >= ENC_Sensitivity){
    enc2Old = enc2New;
    return difference / ENC_Sensitivity;
  }else{
    return 0;
  }
}


/*
Name: resetMenu
Description: Resets all the global selector variables for the UI's menu.
Returns: Nothing (edits global variables)
Parameters: None
*/
void resetMenu(){
  menuSelected = 0;
  menuSelecting = 0;
  chDataSelected = 0;
  chDataSelecting = 0;
}


/*
Name: updateButton1
Description: Updates the menu selection process based on the value of button 1. (Per cycle button 1 check)
Returns: Nothing (edits global variables)
Parameters: None
*/
void updateButton1(){  
  if(checkButton1() == true){
    //Open menu 
    if(showMenu == 0){
      showMenu = 1;
        return;
    }

    // Close menu
    if(menuSelected == 0){
      showMenu = 0;
      resetMenu();
      return;
    }
    
    // Go back to "main menu" unless "channels" is selected, then go back to "main channels menu"
    if(menuSelected > 0){
      if(menuSelected == 1){
        if(chDataSelected > 0){
          chDataSelected = 0;
          chDataSelecting = 0;
          return;
        }else{
          resetMenu();
        }
      }else{
        resetMenu();
        return;
      }
    }
  }
}

/*
Name: updateTrigger
Description: Updates the trigger voltage's value based on an inputted number of increments (increments being read from the UI).
Returns: Nothing (edits global variables)
Parameters: int "increments"
*/
void updateTrigger(int increments){
  triggerVoltage += increments*TRIGGER_Sensitivity;

  bound(triggerVoltage, 0, MAX_TRIGGER);
}

/*
Name: updateHScale
Description: Updates the horizontal scale's value based on an inputted number of increments (increments being read from the UI).
Returns: Nothing (edits global variable)
Parameters: int "increments"
*/
void updateHScale(int increments){
  HScale += increments*HSCALE_Sensitivity;

  bound(HScale, MIN_HSCALE, MAX_HSCALE);
}

/*
Name: updateVScale
Description: Updates the vertical scale's value based on an inputted number of increments (increments being read from the UI).
Returns: Nothing (edits global variable)
Parameters: int "increments"
*/
void updateVScale(int increments){
  VScale += increments*VSCALE_Sensitivity;

  bound(VScale, 0, MAX_VSCALE);
}

/*
Name: updateUI
Description: The central UI function. Uses a switchcase to determine which global variable is currently selected for editing. To the user, this is what
allows them to edit the oscilloscope's control values (ex: trigger voltage, or whether or not to display a waveform)
Returns: Nothing (edits global variables)
Parameters: None
*/
void updateUI(){
  switch(menuSelected){
    
    case 0: //"General Menu"
      menuSelecting += readEncoder2Change();
      menuSelecting = menuSelecting % 4; // Bound the selector to be values 0-3 because there are only 0-3 options
      if(checkButton2() == true){
        menuSelected = menuSelecting;
      }
    break;

    case 1: //"Channels Menu"
      switch(chDataSelected){
        case 0: //"Channels Menu"
          chDataSelecting += readEncoder2Change();
          chDataSelecting = chDataSelecting % 5; // Bound the selector to be values 0-4 because there are only 0-4 options
          if(checkButton2() == true){
          chDataSelected = chDataSelecting;
          }
        break;
        case 1: // "Show Wave 1"
          if(checkButton2() == true){
            if(showWave1 == true){
              showWave1 = false;
            }else{
              showWave1 = true;
            }
          }
        break;
        case 2: // "Show wave 2"
          if(checkButton2() == true){
            if(showWave2 == true){
              showWave2 = false;
            }else{
              showWave2 = true;
            }
          }
        break;
        case 3: // "Show CH1 measurements"
          if(checkButton2() == true){
            if(showMeas1 == true){
              showMeas1 = false;
            }else{
              showMeas1 = true;
            }
          }
        break;
        case 4: // "Show CH2 measurements"
          if(checkButton2() == true){
            if(showMeas2 == true){
              showMeas2 = false;
            }else{
              showMeas2 = true;
            }
          }
        break;
        default: // "General Menu"
          menuSelecting += readEncoder2Change();
          menuSelecting = menuSelecting % 4; // Bound the selector to be values 0-3 because there are only 0-3 options
          if(checkButton2() == true){
            menuSelecting = menuSelected;
          }
      }
    break;

    case 2: // "Trigger voltage selection"
      updateTrigger(readEncoder2Change());
    break;
    case 3: // "Scaling selection"
      updateVScale(readEncoder1Change());
      updateHScale(readEncoder2Change());
    break;
  }

  updateButton1();
}


/*
Name: UITerninalTest
Description: Used for testing & debugging. Replaces the oscilloscope's screen with terminal print outs of the menu. (For use in DUMMY mode)
Returns: Nothing (prints to terminal)
Parameters: None
*/
void UITerminalTest(){
  Serial.println("------------------- UI Test ----------------------");

  updateButton1();
  
  #if DUMMY
  if(showMenu == true){
  Serial.println("_____MENU_____");
  Serial.print("Select-ed: ");
  switch(menuSelected){
    case 0:
    Serial.println("Menu");
    break;

    case 1:
    Serial.println("Channels");
    Serial.print("dataSelect-ed: ");
    switch(chDataSelected){
      case 0:
      Serial.println("Channel Menu");
      break;
      
      case 1:
      Serial.println("showWave1");
      break;

      case 2:
      Serial.println("showWave2");
      break;

      case 3:
      Serial.println("showMeas1");
      break;

      case 4:
      Serial.println("showMeas2");
      break;
    }
    Serial.print("dataSelect-ing: ");
    switch(chDataSelecting){
      case 0:
      Serial.println("Channel Menu");
      break;
      
      case 1:
      Serial.println("showWave1");
      break;

      case 2:
      Serial.println("showWave2");
      break;

      case 3:
      Serial.println("showMeas1");
      break;

      case 4:
      Serial.println("showMeas2");
      break;
    }

    break;

    case 2:
    Serial.println("Trigger Voltage");
    break;
    
    case 3:
    Serial.println("Scaling");
    break;
  }

  Serial.print("Select-ing: ");
  switch(menuSelecting){
    case 0:
    Serial.println("Menu");
    break;

    case 1:
    Serial.println("Channels");
    break;

    case 2:
    Serial.println("Trigger Voltage");
    break;
    
    case 3:
    Serial.println("Scaling");
    break;
  }

  Serial.println("CURRENT VALUES:");
  Serial.print("Trigger Voltage: ");
  Serial.println(triggerVoltage);
  Serial.print("H_Scale: ");
  Serial.println(HScale);
  Serial.print("V_Scale: ");
  Serial.println(VScale);
  Serial.print("showWave1: ");
  Serial.println(showWave1);
  Serial.print("showWave2: ");
  Serial.println(showWave2);
  Serial.print("showMeas1: ");
  Serial.println(showMeas1);
  Serial.print("showMeas2: ");
  Serial.println(showMeas2);
  updateUI();
}else{

}
#endif
}




//--------------------------
/* END  UI Functions */
// -------------------------
#endif



// ----------------------
/* BEGIN ADC FUNCTIONS */
// ----------------------

/*
Name: sampleChannels
Description: Sample ADC0 and ADC1 on the Teensy 4.1 (for channel 1 & 2) 4000 times, and store it in an array of unsigned 16-bit values.
The 16-bit value was chosen because the ADC has 10-bit resolution, and 16-bit was the next largest value for storing it.
Returns: Nothing (updates global arrays)
Parameters: None
*/
void sampleChannels(){
  #if DUMMY
    for(int i = 0; i < NUM_SAMPLES; i++){
      rawData1[i] = (int)(i%1023);
      rawData2[i] = (int)(1023 - i%1023);
    }
  #endif

  #if !DUMMY
  for(int i = 0; i < NUM_SAMPLES; i++){
  while(adc->adc0->isConverting() || adc->adc1->isConverting());

  uint16_t ch1Val = adc->adc0->readSingle();
  uint16_t ch2Val = adc->adc1->readSingle();

  rawData1[i] = ch1Val;
  rawData2[i] = ch2Val;

  adc->startSynchronizedSingleRead(CH1_PIN, CH2_PIN);
  }
  #endif
}

/*
Name: updateVoltageData
Description: Update the global 1D arrays in units volts for channel 1 and channel 2. (i.e. convert the 10-bit
ADC value into a value representing the real-world voltage being sampled, and store it in the corresponding arrays). Additionally,
determine where in the signal data the trigger voltage first appears, and store this index for read-back later.
Returns: Nothing (updates global arrays)
Parameters: None
*/
void updateVoltageData(){
  sig1TrigIndex = 0;
  sig2TrigIndex = 0;

  for(int i = 0; i < NUM_SAMPLES; i ++){
    // Take the fraction of the full range that the value represents (raw/1023), multiply it by full voltage range, add the lower bound
    voltageData1[i] = upperVoltage - ((rawData1[i]*1.0)/1023)*(upperVoltage - lowerVoltage);
    voltageData2[i] = upperVoltage - ((rawData2[i]*1.0)/1023)*(upperVoltage - lowerVoltage);

    // Determine where the trigger voltage starts in the waveform (with +/- 5% error allowed for trigger voltage)
    if(sig1TrigIndex == 0 && (voltageData1[i] > (0.95*triggerVoltage) && voltageData1[i] < (1.05*triggerVoltage))){
      sig1TrigIndex = i;
    }
    if(sig2TrigIndex == 0 && (voltageData2[i] > (0.95*triggerVoltage) && voltageData2[i] < (1.05*triggerVoltage))){
      sig2TrigIndex = i;
    }
  }
}

/*
Name: extractPlottingData
Description: Using the horizontal scale (HScale) and the time-per-sample (smapleDt), index the NUM_SAMPLES length array 
to extract 320 points for plotting on the 320-pixel wide TFT display.
Returns: Nothing (updates global arrays)
Parameters: None
*/
void extractPlottingData(){
  int indexRange = 320;
  double stride = 0;
  double strideIndex = 0;;
  
  indexRange = (int)((32.0*HScale)/sampleDt);

  bound(indexRange, 1, NUM_SAMPLES);

  Serial.print("indexRange: ");
  Serial.println(indexRange);
  Serial.print("Stride: ");
  Serial.println(stride);

  stride = indexRange/320.0;

  for(int i = 0; i < 320; i++){

    if(strideIndex >= NUM_SAMPLES){
      Serial.println("Attempt to index past data array! - Stopped");
      strideIndex = NUM_SAMPLES - 1;
      break;
    }

    sig1Data[i] = voltageData1[(sig1TrigIndex + (int)(strideIndex))%NUM_SAMPLES];
    sig2Data[i] = voltageData2[(sig2TrigIndex + (int)(strideIndex))%NUM_SAMPLES];

    strideIndex += stride;
    
  }
}

/*
Name: printRawChannelData
Description: Used for testing & debugging. Prints out the first 20 values of the NUM_SAMPLES-long "rawData" arrays (both channels)
Returns: Nothing
Parameters: None
*/
void printRawChannelData(){
  Serial.println("Channel 1 Data (first 20 values):");
  Serial.print("[");
  for(int i = 0; i < 20; i++){
    Serial.print(rawData1[i]);
    if(i == NUM_SAMPLES - 1){
      Serial.println("]");
    }else{
      Serial.print(", ");
    }
  }
  
  Serial.println("Channel 2 Data (first 20 values):");
  Serial.print("[");
  for(int i = 0; i < 20; i++){
    Serial.print(rawData2[i]);
    if(i == 19){
      Serial.println("]");
    }else{
      Serial.print(", ");
    }
  }
}


/*
Name: printPlottingData
Description: Used for testing & debugging. Prints out all 320 values of the "plotting data" arrays (both channels).
Returns: Nothing
Parameters: None
*/
void printPlottingData(){
  Serial.println("Channel One Plotting Data -----------");
    Serial.print("[");
    for(int i = 0; i < 320; i++){
      Serial.print(sig1Data[i]);
      if(i == 319){
        Serial.println("]");
      }else{
        Serial.print(", ");
      }
    }

    Serial.println("Channel Two Plotting Data -----------");
    Serial.print("[");
    for(int i = 0; i < 320; i++){
      Serial.print(sig2Data[i]);
      if(i == 319){
        Serial.println("]");
      }else{
        Serial.print(", ");
      }
    }
}


void updateOffsets(){
  double sum1 = 0;
  double sum2 = 0;
  for(int i = 0; i < NUM_SAMPLES; i++){
    sum1 += voltageData1[i];
    sum2 += voltageData2[i];
  }

  offset1 = (sum1*1.0) / NUM_SAMPLES;
  offset2 = (sum2*1.0) / NUM_SAMPLES;
}

// ----------------------
/* END ADC FUNCTIONS */
// ----------------------





//--------------------------
/* BEGIN Display Functions */
// -------------------------

/*
Name: intToCharArr
Description: Converts an integer-type value into an array of chars to allow printing of numbers.
Returns: A char pointer (char array) representing an integer value
Parameters: int "num"
*/
const char* intToCharArr(int num){
  const char* charArr;
  String stringNum = (String)(num);
  charArr = stringNum.c_str();
  return charArr;
}

/*
Name: doubletoCharArr 
Description: Converts a double-type value into an array of chars to allow printing of numbers
Returns: a char pointer (char array) representing a double value
Parameters: double "num"
*/
const char* doubleToCharArr(double num){
  const char* charArr;
  String stringNum = (String)(num);
  charArr = stringNum.c_str();
  return charArr;
}

/*
Name: calcCH1P2P
Description: calcCH1P2P = "calculate channel one peak-to-peak." Find's the highest and lowest values from the 320-value long
channel one plotting array, and subtracts them from eachother to get the maximum voltage difference of the waveform.
Returns: Nothing (updates global variable)
Parameters: None
*/
void calcCH1P2P(){
  double lowV = 10000;
  double highV = -10000;
  for(int i = 0; i < LX; i++){
    if(sig1Data[i] < lowV){
      lowV = sig1Data[i];
    }
    if(sig1Data[i] > highV){
      highV = sig1Data[i];
    }
  }

  CH1_P2P = highV - lowV;
}


/*
Name: calcCH2P2P
Description: calcCH1P2P = "calculate channel two peak-to-peak." Find's the highest and lowest values from the 320-value long
channel two plotting array, and subtracts them from eachother to get the maximum voltage difference of the waveform.
Returns: Nothing (updates global variable)
Parameters: None
*/
void calcCH2P2P(){
  double lowV = 10000;
  double highV = -10000;
  for(int i = 0; i < LX; i++){
    if(sig2Data[i] < lowV){
      lowV = sig2Data[i];
    }
    if(sig2Data[i] > highV){
      highV = sig2Data[i];
    }
  }

  CH2_P2P = highV - lowV;
}

/*
Name: calcCH1T
Description: Calculates the period of channel one. (1) Determines if the waveform is increasing or decreasing from the first point. (2) waits until the value
'comes back around', which would be half a period. (3) Update the CH1_T value with the calculate period. Update the value to -1 if a period couldn't be found.
Returns: Nothing (updates global variable)
Parameters: None
*/
void calcCH1T(){
  
  #if dummy 
    CH1_T = 999;
  #endif
  
  #if !dummy
  double sample;
  int repeatIndex = -1; 
  int slope;
  int signCounter = 0;

  sample = voltageData1[0];

  // Determine the waveform's slope direction (positive or negative)
  for(int i = 1; i < NUM_SAMPLES; i++){
    if(voltageData1[i] < sample){
      signCounter--;
    }else if(voltageData1[i] > sample){
      signCounter++;
    }else{

    }
    
    if(abs(signCounter) >= 3){ // If three or more repeated in/decreases of value are detected, slope sign can be determined
      if(signCounter < 0){
        slope = -1;
        break;
      }else if(signCounter > 0){
        slope = 1;
        break;
      }
    }
  }


  // Using the slope direction, look for when the value 'comes back around', and record the index where it does
  for(int i = 1; i < NUM_SAMPLES; i++){
    if(slope < 0){
      if(voltageData1[i] > sample){
        repeatIndex = i;
        break;
      }
    }
    if(slope > 0){
      if(voltageData1[i] < sample){
        repeatIndex = i;
        break;
      }
    }
  }

  // Calculate the period. Set the period to -1 if it could not be determined
  if(repeatIndex < 0){
    CH1_T = -1;
  }else{
    CH1_T = (repeatIndex*2.0)*sampleDt;
  }

  #endif
}

/*
Name: calcCH1T
Description: Calculates the period of channel one. (1) Determines if the waveform is increasing or decreasing from the first point. (2) waits until the value
'comes back around', which would be half a period. (3) Update the CH1_T value with the calculate period. Update the value to -1 if a period couldn't be found.
Returns: Nothing (updates global variable)
Parameters: None
*/
void calcCH2T(){
  
  #if dummy 
    CH2_T = 999;
  #endif
  
  #if !dummy
  double sample;
  int repeatIndex = -1; 
  int slope;
  int signCounter = 0;

  sample = voltageData2[0];

  // Determine the waveform's slope direction (positive or negative)
  for(int i = 1; i < NUM_SAMPLES; i++){
    if(voltageData2[i] < sample){
      signCounter--;
    }else if(voltageData2[i] > sample){
      signCounter++;
    }else{

    }
    
    if(abs(signCounter) >= 3){ // If three or more repeated in/decreases of value are detected, slope sign can be determined
      if(signCounter < 0){
        slope = -1;
        break;
      }else if(signCounter > 0){
        slope = 1;
        break;
      }
    }
  }


  // Using the slope direction, look for when the value 'comes back around', and record the index where it does
  for(int i = 1; i < NUM_SAMPLES; i++){
    if(slope < 0){
      if(voltageData2[i] > sample){
        repeatIndex = i;
        break;
      }
    }
    if(slope > 0){
      if(voltageData2[i] < sample){
        repeatIndex = i;
        break;
      }
    }
  }

  // Calculate the period. Set the period to -1 if it could not be determined
  if(repeatIndex < 0){
    CH2_T = -1;
  }else{
    CH2_T = (repeatIndex*2.0)*sampleDt;
  }

  #endif
}

/*
Name: drawAxes
Description: Uses the TGX library to draw a collection of vertical & horizontal lines, making the oscilloscope's 
X & Y axes.
Returns: Nothing (shows on display)
Parameters: None
*/
void drawAxes()
  {
    // Draw X-axis
    oScopeImage.drawFastHLine(tgx::iVec2 {0, 120}, 320, WHITE);

    //Draw Y-axis
    oScopeImage.drawFastVLine(tgx::iVec2 {160, 0}, 240, WHITE);
   
    // Draw small markers along the X-axis
    for(int i = 0; i <= 32; i++){
      oScopeImage.drawFastVLine(tgx::iVec2 {i*10, 115}, 10, WHITE);
    }

    // Draw small markers along the Y-axis
    for(int i = 0; i <= 24; i++){
      oScopeImage.drawFastHLine(tgx::iVec2 {155, i*10}, 10, WHITE);
    }
  }


/*
Name: displayTriggerVoltage
Description: Reads the global "triggerVoltage" variable and displays it on screen. Most accurate units are also printed (mV or V).
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayTriggerVoltage(){
    
    oScopeImage.drawText("Volt Trig: ", {240, 10}, TRIG_VOLT_FONT, WHITE); // Display "Volt Trig" heading
    
    if(abs(triggerVoltage) < 1){
      oScopeImage.drawText(intToCharArr((int)(triggerVoltage*1000)), {285, 10}, TRIG_VOLT_FONT, WHITE); // Display trigger voltage value
      oScopeImage.drawText("mV", {305, 10}, TRIG_VOLT_FONT, WHITE); // Add "mV" units
    } else {
      oScopeImage.drawText(doubleToCharArr(triggerVoltage), {285, 10}, TRIG_VOLT_FONT, WHITE); // Display trigger voltage value
      oScopeImage.drawText("V", {305, 10}, TRIG_VOLT_FONT, WHITE); // Add "V" units
    }
    
  }


/*
Name: displayHScale
Description: Reads the global "HScale" variable and displays its value on screen.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayHScale(){
    // On-screen positions are hard-coded here for our given display arrangement
    oScopeImage.drawText("Horz: ", {265, 230}, SCALE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(HScale*1000000.0), {295, 230}, SCALE_FONT, WHITE);
  }


/*
Name: displayVScale
Description: Reads the global variable "VScale" and displays its value on screen.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayVScale(){
    // On-screen positions are hard-coded here for our given display arrangement
    oScopeImage.drawText("Vert: ", {200, 230}, SCALE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(VScale), {230, 230}, SCALE_FONT, WHITE);
  }


  void displayOffsets(){
    updateOffsets();
    // On-screen positions are hard-coded here for our given display arrangement
    oScopeImage.drawText("Offset1: ", {220, 120}, SCALE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(offset1*1.0), {280, 120}, SCALE_FONT, WHITE);
    oScopeImage.drawText("Offset2: ", {220, 140}, SCALE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(offset2*1.0), {280, 140}, SCALE_FONT, WHITE);
  }

/*
Name: displayCH1Meas
Description: displayCH1Meas = "display channel one measurements." Calculates and displays channel one's peak-to-peak voltage and period.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayCH1Meas(){
    calcCH1P2P();
    calcCH1T();
    
    // On-screen positions are hard-coded here for our given display arrangement
    oScopeImage.drawText("CH1 Measurements:", {0,10}, MEAS_FONT, CH1_COLOR);
    oScopeImage.drawText("P2P:", {0,25}, MEAS_FONT, CH1_COLOR);
    oScopeImage.drawText("T:", {0,40}, MEAS_FONT, CH1_COLOR);

    if(abs(CH1_P2P) < 1){
      oScopeImage.drawText(intToCharArr((int)(CH1_P2P*1000)), {25,25}, MEAS_FONT, CH1_COLOR);
      oScopeImage.drawText("mV", {45, 25}, TRIG_VOLT_FONT, CH1_COLOR); // Add "mV" units
    } else {
      oScopeImage.drawText(doubleToCharArr(CH1_P2P), {25,25}, MEAS_FONT, CH1_COLOR);
      oScopeImage.drawText("V", {45, 25}, TRIG_VOLT_FONT, CH1_COLOR); // Add "V" units
    }

    if(abs(CH1_T) < 0.001){
      oScopeImage.drawText(intToCharArr((int)(CH1_T*1000000)), {15,40}, MEAS_FONT, CH1_COLOR);
      oScopeImage.drawText("us", {35, 40}, TRIG_VOLT_FONT, CH1_COLOR); // Add "micro-seconds" units
    } else if (abs(CH1_T) < 1){
      oScopeImage.drawText(intToCharArr((int)(CH1_T*1000)), {15,40}, MEAS_FONT, CH1_COLOR);
      oScopeImage.drawText("ms", {35, 40}, TRIG_VOLT_FONT, CH1_COLOR); // Add "milli-seconds" units
    } else {
      oScopeImage.drawText(doubleToCharArr(CH1_T), {15,40}, MEAS_FONT, CH1_COLOR);
      oScopeImage.drawText("s", {35, 40}, TRIG_VOLT_FONT, CH1_COLOR); // Add "seconds" units
    }
  }


/*
Name: displayCH2Meas
Description: displayCH2Meas = "display channel two measurements." Calculates and displays channel two's peak-to-peak voltage and period.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayCH2Meas(){
    calcCH2P2P();
    calcCH2T();

    // On-screen positions are hard-coded here for our given display arrangement
    oScopeImage.drawText("CH2 Measurements:", {0,205}, MEAS_FONT, CH2_COLOR);
    oScopeImage.drawText("P2P:", {0,220}, MEAS_FONT, CH2_COLOR);
    oScopeImage.drawText("T:", {0,235}, MEAS_FONT, CH2_COLOR);
    
    if(abs(CH2_P2P) < 1){
      oScopeImage.drawText(intToCharArr((int)(CH2_P2P*1000)), {25,220}, MEAS_FONT, CH2_COLOR);
      oScopeImage.drawText("mV", {45, 220}, TRIG_VOLT_FONT, CH2_COLOR); // Add "mV" units
    } else {
      oScopeImage.drawText(doubleToCharArr(CH2_P2P), {25,220}, MEAS_FONT, CH2_COLOR);
      oScopeImage.drawText("V", {45, 220}, TRIG_VOLT_FONT, CH2_COLOR); // Add "V" units
    }

    if(abs(CH2_T) < 0.001){
      oScopeImage.drawText(intToCharArr((int)(CH2_T*1000000)), {15, 235}, MEAS_FONT, CH2_COLOR);
      oScopeImage.drawText("us", {35, 235}, TRIG_VOLT_FONT, CH2_COLOR); // Add "micro-seconds" units
    } else if (abs(CH2_T) < 1){
      oScopeImage.drawText(intToCharArr((int)(CH2_T*1000)), {15, 235}, MEAS_FONT, CH2_COLOR);
      oScopeImage.drawText("ms", {35, 235}, TRIG_VOLT_FONT, CH2_COLOR); // Add "milli-seconds" units
    } else {
      oScopeImage.drawText(doubleToCharArr(CH2_T), {15, 235}, MEAS_FONT, CH2_COLOR);
      oScopeImage.drawText("s", {35, 235}, TRIG_VOLT_FONT, CH2_COLOR); // Add "seconds" units
    }
  }

/*
Name: displayCH1Signal
Description: displayCH1Signal = "display channel one's signal (waveform)." Map all of channel one's 320 extracted voltage data points
into x & y vectors (2D coordinates), then plot them on screen. Color of the points are CH1_COLOR
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayCH1Signal(){

    // Translate & scale channel 1's voltage values to pixel coordinate (for plotting)
    for(int i = 0; i < LX; i++){
      sig1Points[i].x = i;
      sig1Points[i].y = (int)((LY/2) + (sig1Data[i]/VScale)*120); // Y pixel coordinate = 120 + (voltage/scalar)*120
    }
    // Plot channel 1's data points (voltage vs time)
    for(int i = 0; i < LX; i++){
      oScopeImage.drawPixel(sig1Points[i], CH1_COLOR);
    }
  }


/*
Name: displayCH2Signal
Description: displayCH2Signal = "display channel two's signal (waveform)." Map all of channel two's 320 extracted voltage data points
into x & y vectors (2D coordinates), then plot them on screen. Color of the points are CH2_COLOR
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayCH2Signal(){

    // Translate & scale channel 2's voltage values to pixel coordinate (for plotting)
    for(int i = 0; i < LX; i++){
      sig2Points[i].x = i;
      sig2Points[i].y = (int)((LY/2) + (sig2Data[i]/VScale)*120); // Y pixel coordinate = 120 + (voltage/scalar)*120
    }

    // Plot channel 2's data points (voltage vs time)
    for(int i = 0; i < LX; i++){
      oScopeImage.drawPixel(sig2Points[i], CH2_COLOR);
    }
  }


/*
Name: displayTriggerSelect
Description: Displays the moscilloscope menu's "trigger select" option for changing the trigger voltage value
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayTriggerSelect(){
    oScopeImage.fillThickRect({110, 210, 0, 40}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);
    
    oScopeImage.drawText("Trig: ", {116, 25}, CHANGE_VALUE_FONT, WHITE); // Display "Volt Trig" heading
    
    if(abs(triggerVoltage) < 1){
      oScopeImage.drawText(intToCharArr((int)(triggerVoltage*1000)), {150, 25}, CHANGE_VALUE_FONT, WHITE); // Display trigger voltage value
      oScopeImage.drawText("mV", {185, 25}, CHANGE_VALUE_FONT, WHITE); // Add "mV" units
    } else {
      oScopeImage.drawText(doubleToCharArr(triggerVoltage), {150, 25}, CHANGE_VALUE_FONT, WHITE); // Display trigger voltage value
      oScopeImage.drawText("V", {185, 25}, CHANGE_VALUE_FONT, WHITE); // Add "V" units
    }

  }

  /*
Name: displayScalingSelect
Description: Displays the moscilloscope menu's "scaling select" option for changing the horizontal and vertical scale values
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayScalingSelect(){
    oScopeImage.fillThickRect({110, 210, 0, 65}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    oScopeImage.drawText("Horz: ", {114, 25}, CHANGE_VALUE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(HScale*1000000.0), {165, 25}, CHANGE_VALUE_FONT, WHITE);
  
    // On-screen positions are hard-coded here for our given display arrangement
    oScopeImage.drawText("Vert: ", {114, 50}, CHANGE_VALUE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(VScale), {165, 50}, CHANGE_VALUE_FONT, WHITE);
  }

/*
Name: displayWave1Select
Description: Displays the moscilloscope menu's "wave 1 select" option for turning on/off channel one's waveform plot
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayWave1Select(){
    oScopeImage.fillThickRect({110, 210, 0, 40}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    oScopeImage.drawText("Wave 1: ", {114, 25}, CHANGE_VALUE_FONT, WHITE); 
    if(showWave1 == true){
      oScopeImage.drawText("ON", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }else{
      oScopeImage.drawText("OFF", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }
  }

  /*
Name: displayWave2Select
Description: Displays the moscilloscope menu's "wave 2 select" option for turning on/off channel two's waveform plot
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayWave2Select(){
    oScopeImage.fillThickRect({110, 210, 0, 40}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    oScopeImage.drawText("Wave 2: ", {114, 25}, CHANGE_VALUE_FONT, WHITE); 
    if(showWave2 == true){
      oScopeImage.drawText("ON", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }else{
      oScopeImage.drawText("OFF", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }
  }


/*
Name: displayMeas1Select
Description: Displays the moscilloscope menu's "measurements 1 select" option for turning on/off channel one's measurements display
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayMeas1Select(){
    oScopeImage.fillThickRect({110, 210, 0, 40}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    oScopeImage.drawText("Meas 1: ", {114, 25}, CHANGE_VALUE_FONT, WHITE); 
    if(showMeas1 == true){
      oScopeImage.drawText("ON", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }else{
      oScopeImage.drawText("OFF", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }
  }

  /*
Name: displayMeas2Select
Description: Displays the moscilloscope menu's "measurements 2 select" option for turning on/off channel two's measurements display
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayMeas2Select(){
    oScopeImage.fillThickRect({110, 210, 0, 40}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    oScopeImage.drawText("Meas 2: ", {114, 25}, CHANGE_VALUE_FONT, WHITE); 
    if(showMeas2 == true){
      oScopeImage.drawText("ON", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }else{
      oScopeImage.drawText("OFF", {175, 25}, CHANGE_VALUE_FONT, WHITE); 
    }
  }


/*
Name: displayMenuSelector
Description: Displays which menu option the user is currently selected to by displaying a red box around their selection
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayMenuSelector(){
    // Make the following rectangle's coordinates dependent on the menu-selecting variable
    if(menuSelecting == 0){
      oScopeImage.drawRect({25, 93, 30, 190}, tgx::RGB32_Red);

    }else{
      oScopeImage.drawRect({32, 86, 61+(menuSelecting-1)*47, 76+(menuSelecting-1)*47}, tgx::RGB32_Red);
    }
  }

/*
Name: displayChannelsSelector
Description: Displays which channel menu (menu >> channels) the user is currently selecting by displaying a red box around their selection
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayChannelsSelector(){
    // Make the following rectangle's coordinates dependent on the channels-selecting variable
    if(chDataSelecting == 0){
      oScopeImage.drawRect({99, 178, 30, 190}, tgx::RGB32_Red);

    }else{
      oScopeImage.drawRect({102, 175, 52+(chDataSelecting-1)*32, 67+(chDataSelecting-1)*32}, tgx::RGB32_Red);
    }
  }

/*
Name: displayChannelsBlock 
Description: Displays the channels menu block with its four options (show wave 1, wave 2, meas 1, meas 2). Calls the selector display function as well.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayChannelsBlock(){
    oScopeImage.fillThickRect({99, 178, 30, 190}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1); // gray filled, 2 pixels thick red rectangle, 0% opacity (main menu box)
    oScopeImage.fillThickRect({102, 175, 52+(0)*32, 67+(0)*32}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);
    oScopeImage.fillThickRect({102, 175, 52+(1)*32, 67+(1)*32}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);
    oScopeImage.fillThickRect({102, 175, 52+(2)*32, 67+(2)*32}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);
    oScopeImage.fillThickRect({102, 175, 52+(3)*32, 67+(3)*32}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    displayChannelsSelector();

    /* Text of each option....*/
    oScopeImage.drawText("Show Wave 1", {105, 64}, MENU_FONT, MENU_COLOR);
    oScopeImage.drawText("Show Wave 2", {105, 96}, MENU_FONT, MENU_COLOR);
    oScopeImage.drawText("Show Meas 1", {105, 128}, MENU_FONT, MENU_COLOR);
    oScopeImage.drawText("Show Meas 2", {105, 160}, MENU_FONT, MENU_COLOR);
  }


/*
Name: displayMenuBlock
Description: Displays the main menu's block of options (Channels, Trigger, and Scaling). Calls the menu selector display function as well.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayMenuBlock(){
    // Switch case needs to happen first to ensure that lower-level selections don't have the menu shown in frame
    
    oScopeImage.fillThickRect({25, 93, 30, 190}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1); // gray filled, 2 pixels thick red rectangle, 0% opacity (main menu box)
    oScopeImage.fillThickRect({32, 86, 61, 76}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);
    oScopeImage.fillThickRect({32, 86, 108, 123}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);
    oScopeImage.fillThickRect({32, 86, 155, 170}, 2, tgx::RGB32_Gray, tgx::RGB32_White, 1);

    displayMenuSelector();

    /* Text of each option....*/
    oScopeImage.drawText("Channels", {37, 73}, MENU_FONT, MENU_COLOR);
    oScopeImage.drawText("Trigger", {41, 120}, MENU_FONT, MENU_COLOR);
    oScopeImage.drawText("Scaling", {41, 167}, MENU_FONT, MENU_COLOR);
  }


/*
Name: displayMenu
Description: The main menu-displaying function. Uses a switchcase that is based on the global menu-selecting variables to display which option the
user is selected on, as well any necessary information related to that option.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayMenu(){
    if(menuSelected > 0){
      //switch case of channels, scaling, and trigger
      switch(menuSelected){
        case 1: //Channels
          if(chDataSelected > 0){
            switch(chDataSelected){
              case 1:
                displayWave1Select();
              break;

              case 2:
                displayWave2Select();
              break;

              case 3:
                displayMeas1Select();
              break;

              case 4:
                displayMeas2Select();
              break;
            }
          }else{
            displayMenuBlock();
            displayChannelsBlock();
          }
        break;

        case 2: // Trigger Voltage Selection
          displayTriggerSelect();
        break;

        case 3:
          displayScalingSelect();
        break;
      }
      
    }else{
      displayMenuBlock();
    }
  }


/*
Name: displayChannels
Description: Depending on the boolean value of the global variables for channel 1 & 2, calls/doesn't call the functions for displaying
the waveforms and measurements of each channel.
Returns: Nothing (shows on display)
Parameters: None
*/
  void displayChannels(){
    if(showMeas1){
      displayCH1Meas();
    }
    if(showMeas2){
      displayCH2Meas();
    }
    if(showWave1){
      displayCH1Signal();
    }
    if(showWave2){
      displayCH2Signal();
    }
  }


  void displayUIStates(){
    button1.update();
    button2.update();
    button3.update();
    button4.update();

    oScopeImage.drawText("E1: ", {170, 100}, SCALE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(encoder1.read()), {200, 100}, SCALE_FONT, WHITE);

    oScopeImage.drawText("E2: ", {170, 120}, SCALE_FONT, WHITE);
    oScopeImage.drawText(doubleToCharArr(encoder2.read()), {200, 120}, SCALE_FONT, WHITE);

    oScopeImage.drawText("B1: ", {170, 140}, SCALE_FONT, WHITE);
    oScopeImage.drawText(intToCharArr(button1.fell()), {200, 140}, SCALE_FONT, WHITE);

     oScopeImage.drawText("B2: ", {170, 160}, SCALE_FONT, WHITE);
    oScopeImage.drawText(intToCharArr(button2.fell()), {200, 160}, SCALE_FONT, WHITE);

     oScopeImage.drawText("B3: ", {170, 180}, SCALE_FONT, WHITE);
    oScopeImage.drawText(intToCharArr(button3.fell()), {200, 180}, SCALE_FONT, WHITE);

     oScopeImage.drawText("B4: ", {170, 200}, SCALE_FONT, WHITE);
    oScopeImage.drawText(intToCharArr(button4.fell()), {200, 200}, SCALE_FONT, WHITE);

}

//--------------------------
/* END Display Functions */
// -------------------------





//--------------------------
/* BEGIN Arduino Framework (setup & loop) */
// -------------------------

void setup(){
  Serial.begin(9600);
  Serial.println("----- Let the fun begin -----");

  
  // ----------- ADC Setup --------------

  // Configure both ADC's on the Teensy for VERY fast sampling using the ADC library
  /* ADC setup code start */
  adc->adc0->setResolution(ADC_RESOLUTION);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setAveraging(ADC_OVERSAMPLING);
  adc->adc1->setResolution(ADC_RESOLUTION);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc1->setAveraging(ADC_OVERSAMPLING);
  adc->startSynchronizedSingleRead(CH1_PIN, CH2_PIN);
  /* ADC setup code end*/

  sampleChannels();
  updateVoltageData();
  extractPlottingData();

  // ------------ ADC Setup^^ -------------
 



  // // ----------- Display Setup --------------

  if (!tft.begin())
  Serial.print("ouch !");
  tft.setRotation(3);
  tft.setFramebuffer(fb_internal); // registers the internal framebuffer
  tft.setDiffBuffers(&diff1, &diff2); // registering the 2 diff buffers. This activates differential update mode
  tft.setRefreshRate(120); // set the display refresh rate around 120Hz
  tft.setVSyncSpacing(2); // enable vsync and set framerate = refreshrate/2 (typical choice)
  tft.update(fb); // push our memory framebuffer fb to be displayed on the screen

  // // ----------- Display Setup^^ --------------


  
  // ----------- UI Setup --------------
  
  encoder1.write(0);
  encoder2.write(0);


  pinMode(ENC_1A, INPUT_PULLUP);
  pinMode(ENC_1B, INPUT_PULLUP);
  pinMode(ENC_2A, INPUT_PULLUP);
  pinMode(ENC_2B, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  

  button1.attach(BUTTON_1_PIN, INPUT_PULLUP);
  button1.interval(5); // 5 ms debounce interval
  button2.attach(BUTTON_2_PIN, INPUT_PULLUP);
  button2.interval(5); // 5 ms debounce interval
  button3.attach(BUTTON_3_PIN, INPUT_PULLUP);
  button3.interval(5); // 5 ms debounce interval
  button4.attach(BUTTON_4_PIN, INPUT_PULLUP);
  button4.interval(5); // 5 ms debounce interval

  #if runUI
  resetMenu();
  #endif

  // ----------- UI Setup^^ ------------



}
    
    
void loop(){

  #if runUI
  updateButton1();
  updateUI();
  // UITerminalTest();
  #endif

  // ----------- ADC Loop ------------
  bound(HScale, HScaleMin, HScaleMax);
  
  // If in regular mode, sample Teensy's two ADCs and process the data into global arrays
  #if !DUMMY

  sampleChannels();
  updateVoltageData();
  extractPlottingData();
 
  #endif

  // If in DUMMY mode, call make the same function calls (which in DUMMY mode will generate fake data), then
  // print out all the necessary test data.
  #if DUMMY
    sampleChannels();
    updateVoltageData();
    extractPlottingData();

    Serial.println("-------- HScale Test --------");
    Serial.print("HScale: ");
    if(HScale < 1E-3){
      Serial.print(HScale*1000000);
      Serial.println(" us");
    }
    else if(HScale < 1){
      Serial.print(HScale*1000);
      Serial.println(" ms");
    }
    
    Serial.print("Trigger Voltage: ");
    Serial.println(triggerVoltage);

    Serial.print("sig1TrigIndex: ");
    Serial.println(sig1TrigIndex);
    Serial.print("sig2TrigIndex: ");
    Serial.println(sig2TrigIndex);

    printRawChannelData();
    printPlottingData();
  #endif

  // ----------- ADC Loop^^ ----------
  

  // --------- Display + UI Loop ----------
  
  oScopeImage.clear(tgx::RGB32_Black); //Clear the image
  
    
  // Display the basic moscilloscope components
  drawAxes();
  displayTriggerVoltage();
  displayVScale();
  displayHScale();
  displayChannels();
  
  
  #if debugging
  // displayOffsets();
  // displayUIStates();
  #endif

  #if runUI
  // If the user (UI) has indicated, display the menu (from navigation)
  if(showMenu){
    displayMenu();
  }else{
  
  }
  #endif
  

  // "Update" the image (send the the newest frame to the display)
  tft.update(fb);

// --------- Display + UI Loop^^ ----------
  
  }


//--------------------------
/* END Arduino Framework (setup & loop) */
// -------------------------





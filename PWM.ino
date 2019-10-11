// Arduino PWM Fan Control Beta by Adrian Black, May 2017
// Sourcecode can be freely used for your own projects

// Needs 4-pin PC fan with PWM control (25khz)

//INFO https://www.youtube.com/watch?v=AEJ8MQyEOBg

// ALso uses I2C 16x2 LCD module

// 3 buttons for UI control

// DHT-22 used to temperature sensing

// Electronics run on 5v but FAN runs on 12. PWM logic signal is 5v. Use voltage regulator or buck
// converter to allow this to work.

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <EEPROM.h>

#define DHTPIN 4        // pin DHT sensor connected to
#define DHTTYPE DHT22   // DHT 11 in use. See library documentation for other types
const int PWMPin = 3;   // FAN PWM output 25khz signal
const int RightSW = 5;      // Right switch
const int MidSW = 6;      // Middle switch
const int LeftSW = 7;      // Left switch

boolean dotprint = LOW; // used to put flashing . on LCD during normal opearationg

boolean RightSWState = LOW;
boolean MidSWState = LOW;
boolean LeftSWState = LOW;

boolean waitHere = HIGH; // used in the menu loop to wait for input

int debouncecounter = 0;       // how many times we have seen new value
int debouncereading;           // the current value read from the input pin
long debouncetime = 0;         // the last time the output pin was sampled
int debounce_count = 10; // number of millis/samples to consider before declaring a debounced input

const int LoopSpeed = 1000;
                        // Controls speed of main loop
int RampSpeed = 1;
                       // Controls the changing fan speed
int TempCal = 0;
int TempMin = 73;
int TempMax = 77;

int FanMin = 30;         // Minimum fan speed 0-79
int FanMax = 79;        // Maximum fan speed 0-79
int CurrentSpeed = 79;   // The Current fan speed duty cycle (0-79)
int DesiredSpeed = 1;   // The desired fan speed
boolean FanStop = HIGH; // HIGH = fan stops when set to 0, LOW = fan doesn't stop
boolean BacklightLED = HIGH;
                        // LCD Display Backlight HIGH = on, LOW = off
boolean RunState = HIGH; // not used
float DisplayDutyCycle; // Duty cycle displayed on LCD
int buttonInput = 0;    // Used to read the button input

unsigned long time;     // used for button repeat
const unsigned long repeatdelay = 500;
                        // 500ms for repeat

// end varibles //

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);                                            

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
void setup() {
   time = millis(); // setup time variable for button pushes
  
   if (BacklightLED) {
     lcd.backlight(); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
   } else {
     lcd.noBacklight();
   }

    // generate 25kHz PWM pulse rate on Pin 3
    pinMode(PWMPin, OUTPUT);   // OCR2B sets duty cycle
    // Set up Fast PWM on Pin 3
    TCCR2A = 0x23;     // COM2B1, WGM21, WGM20 
    // Set prescaler  
    TCCR2B = 0x0A;   // WGM21, Prescaler = /8
    // Set TOP and initialize duty cycle to zero(0)
    OCR2A = 79;    // TOP DO NOT CHANGE, SETS PWM PULSE RATE
    OCR2B = CurrentSpeed;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0 :(
    
    lcd.begin(16, 2);
    lcd.clear();

    // Load values from EEPROM
    eepromRead(); // Restore values from EEPROM

  lcd.clear();
}

void loop() {

  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  f = f + TempCal; // Apply sensor calibration

 if (f < TempMin) {
  if (FanStop) {
    DesiredSpeed = 0;
  } else {
    DesiredSpeed = FanMin;
  }
 }
 if (f > TempMax) {
  DesiredSpeed = FanMax;
 }

 if (f <= TempMax && f >= TempMin) {
  DesiredSpeed = map(f, TempMin, TempMax, FanMin, FanMax); // Map the Fan Speed to the Duty Cycle
 }


 if ((DesiredSpeed < CurrentSpeed) && ((CurrentSpeed - RampSpeed) > RampSpeed) ) {
  CurrentSpeed = CurrentSpeed - (1 * RampSpeed);
 } else if (DesiredSpeed < CurrentSpeed) {
  CurrentSpeed = CurrentSpeed - 1;
 }

 if ((DesiredSpeed > CurrentSpeed) && ((CurrentSpeed + RampSpeed) > RampSpeed) ) {
  CurrentSpeed = CurrentSpeed + (1 * RampSpeed);
 } else if (DesiredSpeed > CurrentSpeed) {
  CurrentSpeed = CurrentSpeed + 1;
 }

 lcd.setCursor(0,0);
 lcd.print("Fan Speed: ");

 if (FanStop) {
  OCR2B = 0;    // Stop the fan entirely
  lcd.print("OFF  ");
 } else {
  OCR2B = CurrentSpeed;    // set duty cycle
 }
 
if (!FanStop) {
 if (CurrentSpeed <= 0) {
   lcd.print("STOP ");
 } else {
   DisplayDutyCycle = CurrentSpeed * 1.2658227;
   lcd.print(DisplayDutyCycle, 1);
   lcd.print("%  ");
 }
}

  if (isnan(h) || isnan(t) || isnan(f)) {
    lcd.setCursor(0,1);
    lcd.print("DHT Read Fail       ");
    delay (500);
    return;
  } else {
   lcd.setCursor(0,1);
   lcd.print("T:"); 
   lcd.print(f,1); // use t instead of f for degrees C
   lcd.print("F H:");
   lcd.print(h,0);
   lcd.print("%  ");
  }
 

 buttonInput = readButtons();
 if (buttonInput == 1) {   // toggle the backlight
    BacklightLED = !BacklightLED;
    EEPROM.write(8,BacklightLED);
   if (BacklightLED) {
      lcd.backlight(); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
    } else {
      lcd.noBacklight();
    }
   }
 else if (buttonInput == 2) {FanStop = !FanStop; EEPROM.write(7,FanStop);} // stop the fan
 else if (buttonInput == 3) {fanMenu(); waitHere = 1;} // enter the setup menu
 else { 
  if (dotprint) {
    lcd.setCursor(15,1);
    lcd.print((char)126);
    dotprint = !dotprint;
  } else {
    lcd.setCursor(15,1);
    lcd.print(" ");
    dotprint = !dotprint;
  }
  delay(LoopSpeed); }
}

void fanMenu() {

    // Menu starts here!
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Fan Control");
    lcd.setCursor(0,1);
    lcd.print("RUN        SETUP");
    delay(1000);
    while (waitHere) {
      buttonInput = readButtons();
      switch (buttonInput) {
      case 0: break;
      case 1: waitHere = 0; return 0;
      case 2: break;
      case 3: waitHere = 0;
      }
      delay(10);
    }
    
      lcd.clear();
      lcd.print("Fan L Spd: ");
      lcd.setCursor(0,1);
      lcd.print("-     +     Next");
      buttonInput = 0;  // Reset button input routing
      while (buttonInput != 3) {
        lcd.setCursor(12,0); lcd.print(FanMin);lcd.print("  ");
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            FanMin--;
            break;
          case 2:
            FanMin++;
            break;
        }
      }

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Fan L Temp: ");
      lcd.setCursor(0,1);
      lcd.print("-     +     Next");
      buttonInput = 0;  // Reset button input routing
      while (buttonInput != 3) {
        lcd.setCursor(12,0); lcd.print(TempMin);lcd.print("  ");
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            TempMin--;
            break;
          case 2:
            TempMin++;
            break;
        }
      }


      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Fan H Spd: ");
      lcd.setCursor(0,1);
      lcd.print("-     +     Next");
      buttonInput = 0;  // Reset button input routing
      while (buttonInput != 3) {
        lcd.setCursor(12,0); lcd.print(FanMax);lcd.print("  ");
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            FanMax--;
            break;
          case 2:
            FanMax++;
            break;
        }
      }
      
      lcd.clear();
      lcd.setCursor(0,0);
  //  lcd.print("                ");
      lcd.print("Fan H Temp: ");
      lcd.setCursor(0,1);
      lcd.print("-     +     Next");
      buttonInput = 0;  // Reset button input routing
      while (buttonInput != 3) {
        lcd.setCursor(12,0); lcd.print(TempMax);lcd.print("  ");
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            TempMax--;
            break;
          case 2:
            TempMax++;
            break;
        }
      }
      
      lcd.clear();
      lcd.setCursor(0,0);
  //  lcd.print("                ");
      lcd.print("Sensor Cal: ");
      lcd.setCursor(0,1);
      lcd.print("-     +     Next");
      buttonInput = 0;  // Reset button input routing
      while (buttonInput != 3) {
        lcd.setCursor(12,0); lcd.print(TempCal);lcd.print("  ");
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            TempCal--;
            break;
          case 2:
            TempCal++;
            break;
        }
      }


      lcd.clear();
      lcd.setCursor(0,0);
  //  lcd.print("                ");
      lcd.print("Spd Cange X: ");
      lcd.setCursor(0,1);
      lcd.print("-     +     Next");
      buttonInput = 0;  // Reset button input routing
      while (buttonInput != 3) {
        lcd.setCursor(13,0); lcd.print(RampSpeed);lcd.print("  ");
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            if (RampSpeed > 1) {RampSpeed--;}
            break;
          case 2:
            RampSpeed++;
            break;
        }
      }
  
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Save settings? ");
      lcd.setCursor(0,1);
      lcd.print("SAVE     DISCARD");
      buttonInput = 0;  // Reset button input routing
      delay(1000); // testing
      while (buttonInput != 1 && buttonInput != 3) {
        lcd.setCursor(12,0); 
        buttonInput = readButtons();
        switch (buttonInput) {
          case 1: 
            eepromSave(); // Save new values to EEPROM
            delay (2000);
            return(0);
          case 3: 
            eepromRead(); // Reread stored values
            delay (2000);
            return(0);
            break;
        }
      }
    }
  

void eepromErase() {
  // routing not used in this program. Just here in case you want to clear the rom.
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Erase EEPROM");
  lcd.setCursor(0,1);
  lcd.print("Size= "); lcd.print(EEPROM.length());
  delay(1000);

  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void eepromSave() {
 lcd.clear();
 lcd.setCursor(0,0);
 lcd.print("Saving to EEPROM");
 EEPROM.write(0, 254); // this is the value I'll be looking for to know this data is correct
 EEPROM.write(1, RampSpeed);
 EEPROM.write(2, TempCal);
 EEPROM.write(3, TempMin);
 EEPROM.write(4, TempMax); 
 EEPROM.write(5, FanMin);
 EEPROM.write(6, FanMax);
 if (FanStop) {
   EEPROM.write(7, 1);
 } else {
   EEPROM.write(7, 0);
 }
 
 if (BacklightLED) {
   EEPROM.write(8, 1);
 } else {
   EEPROM.write(8, 0);
 }

 if (RunState) {
   EEPROM.write(9, 1);
 } else {
   EEPROM.write(9, 0);
 }

 lcd.setCursor(0,1); 
 lcd.print("Done!");
 delay(1000);
}

void eepromRead() {
 lcd.clear();
 lcd.setCursor(0,0);
 lcd.print("Reading EEPROM");

  if (EEPROM.read(0) == 254) {
    RampSpeed = EEPROM.read(1);
    TempCal = EEPROM.read(2);
    TempMin = EEPROM.read(3);
    TempMax = EEPROM.read(4);
    FanMin = EEPROM.read(5);
    FanMax = EEPROM.read(6);
    FanStop = EEPROM.read(7);
    BacklightLED = EEPROM.read(8);
    RunState = EEPROM.read(9);
    delay(500);
    if (BacklightLED) {
        lcd.backlight(); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
      } else {
        lcd.noBacklight();
      }
    lcd.setCursor(0,1); 
    lcd.print("Done!");
    delay(1000);
  } else {
    lcd.setCursor(0,1); 
    lcd.print("Defaults used.");
    delay(1000);
  }
}

int readButtons() {
  // If we have gone on to the next millisecond
  // const int RightSW = 5;      // Right switch or Switch 0
  // const int MidSW = 6;      // Middle switch or Switch 1
  // const int LeftSW = 7;      // Left switch or Switch 2
  // boolean RightSWState = LOW;
  // boolean MidSWState = LOW;
  // boolean LeftSWState = LOW;
//  int debouncecounter = 0;       // how many times we have seen new value
// int debouncereading;           // the current value read from the input pin
// long debouncetime = 0;         // the last time the output pin was sampled
// int debounce_count = 10; // number of millis/samples to consider before declaring a debounced input

//unsigned long time;     // used for button repeat
//const unsigned long repeatdelay = 2000;

  

  // Right Button
 if (digitalRead(RightSW) && (millis() - time >= repeatdelay)) {
  for(int i = 0; i < 10; i++)
    {
      debouncereading = digitalRead(RightSW);

      if(!debouncereading && debouncecounter > 0)
      {
        debouncecounter--;
      }
      if(debouncereading)
      {
        debouncecounter++; 
      }
      // If the Input has shown the same value for long enough let's switch it
      if(debouncecounter >= debounce_count)
      {
        time = millis();
        debouncecounter = 0;
        RightSWState = 1;
      }
    delay (10); // wait 10ms
    }
   } else {
    RightSWState = 0;
   }
   
  
 if (digitalRead(MidSW) && (millis() - time >= repeatdelay)) {
  for(int i = 0; i < 10; i++)
    {
      debouncereading = digitalRead(MidSW);

      if(!debouncereading && debouncecounter > 0)
      {
        debouncecounter--;
      }
      if(debouncereading)
      {
        debouncecounter++; 
      }
      // If the Input has shown the same value for long enough let's switch it
      if(debouncecounter >= debounce_count)
      {
        time = millis();
        debouncecounter = 0;
        MidSWState = 1;
      }
    delay (10); // wait 10ms
    }
   } else {
    MidSWState = 0;
   }

 if (digitalRead(LeftSW) && (millis() - time >= repeatdelay)) {
  for(int i = 0; i < 10; i++)
    {
      debouncereading = digitalRead(LeftSW);

      if(!debouncereading && debouncecounter > 0)
      {
        debouncecounter--;
      }
      if(debouncereading)
      {
        debouncecounter++; 
      }
      // If the Input has shown the same value for long enough let's switch it
      if(debouncecounter >= debounce_count)
      {
        time = millis();
        debouncecounter = 0;
        LeftSWState = 1;
      }
    delay (10); // wait 10ms
    }
   } else {
    LeftSWState = 0;
   }

  if (RightSWState) { return 1; }
  else if (MidSWState) { return 2; }
  else if (LeftSWState) { return 3; }
  else { return 0; }
}
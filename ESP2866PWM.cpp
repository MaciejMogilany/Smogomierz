                highest frequency / shortest period pin pulse generate / detect test
//  https://stackoverflow.com/questions/42112357/how-to-implement-esp8266-5-khz-pwm
//     uses raw ESP8266 / NodeMCU V1.0 hardware primitive interface of Arduino IDE (no included libraries)
//
//     timing dependencies: WDT, WiFi, I2S, I2C, one wire, UART, SPI, ...
//
//  Arduino GPIO   16    5    4    0    2   14   12   13   15    3    1      0  1  2  3  4  5 ... 12 13 14 15 16 
//  NodeMCU D pin   0    1    2    3    4    5    6    7    8    9   10      3 10  4  9  2  1 ...  6  7  5  8  0 
//                  |    |    |    |    |    |    |    |    |    |    |                      
//    a           WAKE   |    |   F    Tx1   |    |   Rx2  Tx2  Rx0  Tx0             
//     k      (NO PWM or |    |    L   blue  |    |    |    |                               
//      a'    interrupt) |  S       A   *  H |  H |  H |    |                     * led's
//       s         red  S    D       S      S    M    S    H       
//                  *    C    A       H      C    I    I    C                    
//                        L    T              L    S    M    S   
//                         K    A              K    O    O              
//                                       └  -  -  -  - └----UART's----┘         
//                      └--I2C--┘            └-----SPI------┘
//
//  rules of engagement are obscure and vague for effects of argument values for the paramters of these functions:
//      analogWriteRange(qap);     analogWriteFreq( HFreq );              analogWrite(pPulse, 1);  
//
//    http://stackoverflow.com/questions/42112357/how-to-implement-esp8266-5-khz-pwm  
//
//   system #defines:  F_CPU  ESP8266_CLOCK
#define  pInt     D1          // HWI pin: NOT D0 ie. GPIO16 is not hardwared interrupt or PWM pin
#define  pPulse   D2          // PWM pulsed frequency source   ...  ditto D0  (note: D4 = blue LED)
#define  countFor 160000000UL 

#define  gmv(p)   #p          //  get macro value
#define  em(p)    gmv(p)      //  evaluate macro
#define  qap      2           //  minimal number of duty cycle levels (0, 50, 100% ) Quick As Possible ...
#define  HFreq    5150 //((long int) F_CPU==80000000L ? 125000 : 250000)        // ... to minimize time of a cycle period
                                     //   max values      ^   and   ^   found empirically
//     had HFreq=126400   with 75 KHz pulse at 80 MHz ESP clock, every 1 sec but proof of evidence lost
#define  infoTxt (String)                                                  \
                  "\n\n\t    PWM pulse test        "                        \   
                  "\n F_CPU:          " em(F_CPU)                          \
                  "\n ESP8266_CLOCK:  " em(ESP8266_CLOCK)                  \
                  "\n PWM \"freq.\":    " + HFreq + "\n"                   \
                  "\n\n   connect " em(pInt) " to " em(pPulse) "\n"        \  
                  "\n\n             raw     MPU   "                        \
                  "  \n frequency  count   cycle  "

long int oc=1, cntr=1;  
unsigned long int tc=0;
void hwISR(){ cntr++; }                                              // can count pulses if pInt <---> to pPulse
void anISR(){  tc=ESP.getCycleCount(); timer0_write( countFor + tc ); oc=cntr; cntr=1; }

void setup() {                                     // need to still confirm duty cycle=50%  (scope it or ...)
   noInterrupts(); 
      Serial.begin(115200); Serial.println(infoTxt);  delay(10);  // Serial.flush(); Serial.end(); // Serial timing?
      analogWriteRange(qap);     analogWriteFreq( HFreq );              analogWrite(pPulse, 1);    // start PWM
      pinMode( pInt,  INPUT );   attachInterrupt(pInt, hwISR, RISING);                             // count pulses
      timer0_isr_init();         timer0_attachInterrupt( anISR );       anISR();                   // 
   interrupts();
}

void loop() {  delay(10);     if (oc==0) return;  
               Serial.println((String)" "+(oc/1000.0*F_CPU/countFor)+" KHz  "+oc+"  "+tc);   oc=0;    }  
//

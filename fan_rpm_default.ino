// Any header files?
#include <TimerOne.h>
#define PIN_SENSE 3 //where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno)
#define DEBOUNCE 10 //0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
// you can also use a low pass filter on the input pin to stablize the measurement
#define FANSTUCK_THRESHOLD 500 //if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM
//Interrupt handler. Stores the timestamps of the last 2 interrupts and handles debouncing


const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10;

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

unsigned long m = 0; //m is elapsed time. Maybe try micros and use bitshifting instead of division
int p = .001;
 int goal = 1300;
 int scaled =  (goal-1000)/7;
const int baudrate = 115200;
unsigned long rpm = 0;
unsigned long loop_iter = 0;
char buffer[40];
// 

//sensing

//Resources
//  https://fdossena.com/?p=ArduinoFanControl/i.md
//  https://projecthub.arduino.cc/tylerpeppy/b5618d48-7ae8-4fc0-bca5-a857c36fcc5e

unsigned long volatile ts1=0,ts2=0;

void tachISR() {
    m = millis();
    if((m-ts2)>DEBOUNCE){
        ts1=ts2;
        ts2=m;
    }
}

//Calculates the RPM based on the timestamps of the last 2 interrupts. Can be called at any time.
void calcRPM(){
    if(ts2!=0) { //Probably make this logic faster with a (I think) select argument operator. Its been awhile since I did C/C++
      if (m-ts2<FANSTUCK_THRESHOLD) {
        rpm = (120000/(ts2-ts1)); //replace millis with micros and use bitshifting for greater performance on division
    } else { 
      rpm = 0;
    }
}
}

// void setup(){
//     pinMode(PIN_SENSE,INPUT_PULLUP); //set the sense pin as input with pullup resistor
//     attachInterrupt(digitalPinToInterrupt(PIN_SENSE),tachISR,FALLING); //set tachISR to be triggered when the signal on the sense pin goes low
//     Serial.begin(9600); //enable serial so we can see the RPM in the serial monitor
// }
// void loop(){
//     delay(100);
//     Serial.print("RPM:");
//     Serial.println(calcRPM());
// }

//configure Timer 1 (pins 9,10) to output 25kHz PWM
// void setupTimer1(){
//     //Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
//     TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
//     TCCR1B = (1 << CS10) | (1 << WGM13);
//     ICR1 = 320;
//     OCR1A = 0;
//     OCR1B = 0;
// }
// //configure Timer 2 (pin 3) to output 25kHz PWM. Pin 11 will be unavailable for output in this mode
// void setupTimer2(){
//     //Set PWM frequency to about 25khz on pin 3 (timer 2 mode 5, prescale 8, count to 79)
//     TIMSK2 = 0;
//     TIFR2 = 0;
//     TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
//     TCCR2B = (1 << WGM22) | (1 << CS21);
//     OCR2A = 79;
//     OCR2B = 0;
// }
// //equivalent of analogWrite on pin 9
// void setPWM1A(float f){
//     f=f<0?0:f>1?1:f;
//     OCR1A = (uint16_t)(320*f);
// }
// //equivalent of analogWrite on pin 10
// void setPWM1B(float f){
//     f=f<0?0:f>1?1:f;
//     OCR1B = (uint16_t)(320*f);
// }
// //equivalent of analogWrite on pin 3
// void setPWM2(float f){
//     f=f<0?0:f>1?1:f;
//     OCR2B = (uint8_t)(79*f);
// }

void setup() {
  
  m = millis();

  pinMode(OC1A_PIN, OUTPUT);

  // Set Timer1 configuration
  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10) = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  // CS(12:10)  = 0b001  (Input clock select = clock/1)

    Serial.begin(baudrate); //enable serial so we can see the RPM in the serial monitor
    
    //setPwmDuty(50);
    Serial.println("----START----");

    // should be 1700
    pinMode(PIN_SENSE,INPUT_PULLUP); //set the sense pin as input with pullup resistor
    attachInterrupt(digitalPinToInterrupt(PIN_SENSE),tachISR,FALLING); //set tachISR to be triggered when the signal on the sense pin goes low

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;


  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;

    //enable outputs for Timer 1
    // pinMode(9,OUTPUT); //1A
    // pinMode(10,OUTPUT); //1B
    // setupTimer1();
    // //enable outputs for Timer 2
    // pinMode(3,OUTPUT); //2
    // setupTimer2();
    // //note that pin 11 will be unavailable for output in this mode!
    // //example...
    // setPWM1A(0.5f); //set duty to 50% on pin 9
    // setPWM1B(0.2f); //set duty to 20% on pin 10
    // setPWM2(0.8f); //set duty to 80% on pin 3


};


// goal between 1000-1700 RPM
// void setRPM(int goal){
  
//   int p = .001;
//   // rescale with f to be within bounds
//   int scaled =  (goal-1000))/700;

//   curr = calcRPM();
//   if (curr != 30000){
//     err = goal-curr;
//   }
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
    // delay(1000);
    // Serial.print("RPM:");
    // Serial.println(calcRPM());

  // let's do a simple P controller to set to a specific RPM

  // Serial.print("RPM:");
  // Serial.println(calcRPM());
  // Serial.println("Set Duty cycle to 0");
  // setPwmDuty(0);  
  // delay(10000);
  // Serial.print("RPM:");
  // Serial.println(calcRPM());
  // Serial.println("Set Duty cycle to 50");
  // setPwmDuty(50); //Change this value 0-100 to adjust duty cycle
  // delay(10000);

  // Serial.print("RPM:");
  // Serial.println(calcRPM());
  // Serial.println("Set Duty cycle to 100");
  // setPwmDuty(100); //Change this value 0-100 to adjust duty cycle
  // delay(10000);

    m=millis();
    p = .001;
    goal = 1300;
  // rescale with f to be within bounds
  // Serial.println(goal-1000); 

    scaled =  (goal-1000)>>3; //DavidM: why 7? if you can use 8, replace div with >>3
  // Serial.print("Initial scaled:");
  // Serial.println(scaled);  


  // int curr = calcRPM();
  // if (curr < 1800 && curr > 0){
  //   int err = curr-goal;
  //   // not exactly a p controller, but best I can do without state management
  //   scaled += err *p;

  //   Serial.print("Curr: ");
  //   Serial.println(curr);
  

  //   // Serial.print("err:");
  //   // Serial.println(err);

  //   // coerce within duty bounds
  //   scaled = max(0,scaled);
  //   scaled = min(scaled,100);

  //   // Serial.print("Set duty:");
  //   // Serial.println(scaled);
  //   setPwmDuty(scaled);
  // }

  // delay(100);
  calcRPM(); //DavidM: put this on an interrupt timer. Note: you don't have to sample at 25khz, try like 100 Hz.
  
  //Serial.print("RPM:");
  sprintf(buffer,'RPM(%u):%2.2d',loop_iter,rpm)
  Serial.println(buffer); //DavidM: put this on an interrupt timer

  loop_iter++;
  delay(100); //delay is :-(

};

void setPwmDuty(byte duty) {
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}



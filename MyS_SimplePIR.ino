/*
Super minimalistic Mysensors node:
PIR input on D2.
Sleep until motion triggers wake up. 
  Then send notification + battery level.
*/

#include <SPI.h>
#include <MySensor.h>  
#include <DHT.h>  

//#define LOG_EN
#ifdef LOG_EN
#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#else
#define log(x) 
#define logln(x) 
#endif


#define CHILD_ID_MOTION 0
#define MOTION_SENSOR_DIGITAL_PIN 2

#define MIN_V 1800
#define MAX_V 3000

MySensor gw;
MyMessage motionMsg(CHILD_ID_MOTION, V_TRIPPED);

bool lastMotionState = false;
long readVcc();

//************************************************************************
// Setup
//************************************************************************
void setup()  
{ 
  gw.begin();
  
  //setClockPrescaler(CLOCK_PRESCALER_1);
  //Serial.begin(28800);
  Serial.begin(9600);
  logln("SimplePIR v.1.0 - Starting up...");

  // Configure HW IO.
  pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);    
  
  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("PIR", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_MOTION, S_MOTION);  
}

//************************************************************************
// Loop
//************************************************************************
void loop()      
{
  while(gw.getNodeId() == 255){
    log(".");
    gw.process();
  };  // Don't do anything until we get a node id!

  //sendBatteryLevel();
  
  bool motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN);
  if(lastMotionState != motion)
  {
    lastMotionState = motion;
    logln("SendMotion");
    gw.send(motionMsg.set(motion?"1":"0"));      
    
    if(motion) sendBatteryLevel();  // Send battery level each time motion is detected.
  }
  
  logln("Sleep...");
  gw.sleep(0, CHANGE, 0); //sleep untill change of pin.
  logln("Woke on Pin Change.");
}


//************************************************************************
void sendBatteryLevel(){
    //Serial.println("readVcc");
    long batLvl = readVcc();
    //Serial.println("SendVcc");
    //int batteryPcnt = min(map(batLvl, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
    int batteryPcnt = map(batLvl, MIN_V, MAX_V, 0, 100); // Convert voltage to percentage - allow >100% !
    gw.sendBatteryLevel(batteryPcnt); // Send battery percentage to gateway
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  logln("WaitAdcConversionDone...");
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//************************************************************************
// Prescaler setting code... Might not be needed.!
//************************************************************************
#define CLOCK_PRESCALER_1   (0x0)
#define CLOCK_PRESCALER_2   (0x1)
#define CLOCK_PRESCALER_4   (0x2)
#define CLOCK_PRESCALER_8   (0x3)
#define CLOCK_PRESCALER_16  (0x4)
#define CLOCK_PRESCALER_32  (0x5)
#define CLOCK_PRESCALER_64  (0x6)
#define CLOCK_PRESCALER_128 (0x7)
#define CLOCK_PRESCALER_256 (0x8)

// Initialize global variable.
static uint8_t __clock_prescaler = (CLKPR & (_BV(CLKPS0) | _BV(CLKPS1) | _BV(CLKPS2) | _BV(CLKPS3)));

inline void setClockPrescaler(uint8_t clockPrescaler) {
  if (clockPrescaler <= CLOCK_PRESCALER_256) {
    // Disable interrupts.
    uint8_t oldSREG = SREG;
    cli();

    // Enable change.
    CLKPR = _BV(CLKPCE); // write the CLKPCE bit to one and all the other to zero

    // Change clock division.
    CLKPR = clockPrescaler; // write the CLKPS0..3 bits while writing the CLKPE bit to zero

    // Copy for fast access.
    __clock_prescaler = clockPrescaler;

    // Recopy interrupt register.
    SREG = oldSREG;
  }
}


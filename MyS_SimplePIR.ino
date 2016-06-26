/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * 
 * DESCRIPTION
 * This sketch provides an example how to implement a humidity/temperature
 * sensor using DHT11/DHT-22 
 * http://www.mysensors.org/build/humidity
 */
 
#include <SPI.h>
#include <MySensor.h>  
#include <DHT.h>  
//#include <prescaler.h>

#define CHILD_ID_MOTION 0
//#define CHILD_ID_TEMP 1
//#define CHILD_ID_HUM 2
//#define CHILD_ID_LIGHT 3

//#define HUMIDITY_SENSOR_DIGITAL_PIN 4
#define MOTION_SENSOR_DIGITAL_PIN 2
//#define RELAY_PIN 3                               
//#define RELAY_ON 0  // GPIO value to write to turn on attached relay
//#define RELAY_OFF 1 // GPIO value to write to turn off attached relay

//unsigned long DHT_INTERVAL = 30000; // Sleep time between reads (in milliseconds)
//unsigned long BAT_INTERVAL = 10000;

#define MIN_V 1800
#define MAX_V 3000

MySensor gw;
//DHT dht;
//float lastTemp;
//float lastHum;
boolean metric = true; 
//MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
//MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage motionMsg(CHILD_ID_MOTION, V_TRIPPED);
//MyMessage relayMessage(CHILD_ID_LIGHT, V_STATUS);
bool wokeOnInterrupt = false;
bool lastMotionState = false;
static unsigned long myMillis;
static unsigned long lWaitMillis;
static unsigned long lBatWaitMillis;

void incomingMessage(const MyMessage &message);
long readVcc();

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




void setup()  
{ 
  gw.begin(incomingMessage, AUTO, false);

  
  //setClockPrescaler(CLOCK_PRESCALER_1);
  //Serial.begin(28800);
  Serial.begin(115200);

  // Configure HW IO.
  //digitalWrite(RELAY_PIN, RELAY_OFF);
  //pinMode(RELAY_PIN, OUTPUT);    
  pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);    
  
  
  //dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);// Auto - Specific =>, DHT11); 

  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("BatteryNode", "0.1");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_HUM, S_HUM);
  gw.present(CHILD_ID_TEMP, S_TEMP);
  gw.present(CHILD_ID_MOTION, S_MOTION);  
  gw.present(CHILD_ID_LIGHT, S_LIGHT);
  
  //metric = gw.getConfig().isMetric;
  
  lWaitMillis = millis() + 1000;  // initial setup
  
  gw.send(relayMessage.set(false));
  
  myMillis = millis();
}

void sendBatteryLevel(){
      lBatWaitMillis += BAT_INTERVAL;  // Set time for next acquisition.
    //Serial.println("readVcc");
    long batLvl = readVcc();
    //Serial.println("SendVcc");
    //int batteryPcnt = min(map(batLvl, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
    int batteryPcnt = map(batLvl, MIN_V, MAX_V, 0, 100); // Convert voltage to percentage - allow >100% !
    gw.sendBatteryLevel(batteryPcnt); // Send battery percentage to gateway
}

void loop()      
{
  while(gw.getNodeId() == 255){
    Serial.print(".");
    gw.process();
  };  // Don't do anything until we get a node id!

  //Serial.println("NrfProcess");
  //gw.process();
  
  /*
  if( (long)( myMillis - lWaitMillis ) >= 0)
  {
    lWaitMillis += DHT_INTERVAL;  // Set time for next acquisition.

    dht.resetTimer();  // Rest dht time, to ensure conversion at what might seem like too short intervals when using powerDown mode, e.g. where timers don't run!
    
    float temperature = dht.getTemperature();
    if (isnan(temperature)) {
        Serial.println("Failed reading temperature from DHT");
    } else if (temperature != lastTemp) {
      lastTemp = temperature;
      if (!metric) {
        temperature = dht.toFahrenheit(temperature);
      }
      gw.send(msgTemp.set(temperature, 1));
      Serial.print("T: ");
      Serial.println(temperature);
    } else {
      Serial.print("Same T: ");
      Serial.println(temperature);
    }
    
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
        Serial.println("Failed reading humidity from DHT");
    } else if (humidity != lastHum) {
        lastHum = humidity;
        gw.send(msgHum.set(humidity, 1));
        Serial.print("H: ");
        Serial.println(humidity);
    } else {
      Serial.print("Same H: ");
      Serial.println(humidity);
    }
  }
  */
  // Send battery level?
  if( (long)( myMillis - lBatWaitMillis ) >= 0)
  {
    sendBatteryLevel();
    /*lBatWaitMillis += BAT_INTERVAL;  // Set time for next acquisition.
    Serial.println("readVcc");
    long batLvl = readVcc();
    Serial.println("SendVcc");
    //int batteryPcnt = min(map(batLvl, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
    int batteryPcnt = map(batLvl, MIN_V, MAX_V, 0, 100); // Convert voltage to percentage - allow >100% !
    gw.sendBatteryLevel(batteryPcnt); // Send battery percentage to gateway*/
  }
  
  bool motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN);
  if(lastMotionState != motion)
  {
    lastMotionState = motion;
    Serial.println("SendMotion");
    gw.send(motionMsg.set(motion?"1":"0"));      
    
    if(motion) sendBatteryLevel();  // Send battery level each time motion is detected.
  }
  
  // Sleep for some time (wake if interrupted.)
  Serial.println("Sleep...");
  //wokeOnInterrupt = gw.sleep(0, CHANGE, 1000); //sleep untill timeout or pi1 rising.
  wokeOnInterrupt = gw.sleep(0, CHANGE, 0); //sleep untill change of pin.
  if(!wokeOnInterrupt) {
    myMillis += 1000;  // Add slept time to internal "myMillis" time. This way we will be able to use "approximate time" and "only" loose up to 4sec if we had interrupt.
    Serial.print("WDT wake up. myMillis = ");
    Serial.println(myMillis);
  } else Serial.println("Woke on Interrupt.");
}

void incomingMessage(const MyMessage &message) {
  Serial.println("MS Message received...");
  if (message.type==V_LIGHT) {
     // Change relay state
     digitalWrite(RELAY_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
     gw.send(relayMessage.set(message.getBool()));
     // Store state in eeprom
//     gw.saveState(message.sensor, message.getBool());
     // Write some debug info
     Serial.print("Incoming change for sensuator:");
     Serial.print(message.sensor);
     Serial.print(", New status: ");
     Serial.println(message.getBool());
   } 
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
  Serial.println("WaitAdcConversionDone...");
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


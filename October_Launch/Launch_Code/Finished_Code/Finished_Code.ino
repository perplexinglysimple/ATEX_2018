//DEBUG will run Serial 
#define DEBUG

#include <Wire.h>
#include <OneWire.h>
#include <SFE_BMP180.h>


#define ISR_timer 1600

//The class and all underlying functions that handle eeprom and storage.
#define NUM_DEVICES 3
#define ADDRESSOFEEPROM 0x50
#define MEMORY_SIZE 0x8000
const uint8_t sizeofpacket = 5; //The size of struct in bytes that is used for each data
const uint8_t numd = 15; //The number of devices
#include "Memory_Storage.h"

//Temp Sensor  -------------------------------------------------------------------------------------------------------------------
//This handles the Temperature Senors
#define TEMP1_PIN 7
#define TEMP2_PIN 6
#define TEMP3_PIN 10
OneWire ds1(TEMP1_PIN);
OneWire ds2(TEMP2_PIN);
//OneWire ds3(TEMP3_PIN); This commented out because the 3rd temperature sensor was removed
float readTEMP1();
float readTEMP2();
//Temp Sensor End  -------------------------------------------------------------------------------------------------------------------

//Pressure Sensor
void MPUSENSOR(memorystorage &store);
long conftol(float a);
float getPressure();
//Pressure Sensor End

//BMP Sensor -------------------------------------------------------------------------------------------------------------------
SFE_BMP180 pressure; //setup for BMP
bool ALT = 0;
double baseline = 0.0; // pressure before first run through
double ALTITUDE = 0.0; // altitude before first run through
//BMP Sensor End -------------------------------------------------------------------------------------------------------------------

void setup() {
  
  #ifdef DEBUG
    Serial.begin(38400); //serial pin reading from
   //BMP Setup Below------------------------------------------------------------------------------------------------
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }
  #endif
  pressure.begin();

  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 65535;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

volatile int count = 0;
volatile bool servo_activated = false;

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 12 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)

 if (servo_activated) {
    return;
  }
 if(count < ISR_timer)
  {
   count++;
  }
  else
   {
    servo_activated = true;
    for (int myAngle=0; myAngle<=180; myAngle++) {
      int pulseWidth = (myAngle * 2.2) + 1500;  // converts angle to microseconds
      digitalWrite(5, HIGH);       // set servo high
      delayMicroseconds(pulseWidth);      // wait a very small amount
      digitalWrite(5, LOW);        // set servo low
      delay(2.5);
      digitalWrite(5, HIGH);
    }
    count = 0;
  }
}

  
void loop(){
  #ifdef DEBUG
    Serial.begin(38400);
  #endif
  memorystorage store;
  bool readtemp = 1;
  int what;
  char read[4];
enum State {BMP, TEMP, TIME, EEP, WAIT} state = WAIT;
while(1)
{
  double Temp_1, Temp_2;
  switch(state){
    case WAIT:
      #ifdef DEBUG
        Serial.println("Waiting for 1s");
      #endif
      for(int i = 0; i < 1; ++i)
        delay(1000); //1000ms * 1s * 1min = 60s.... 1000ms = 1s
      state = BMP;
      break;
  case BMP://device number 0x0
    float _pread; 
    _pread = getPressure();
    #ifdef DEBUG
      Serial.println("Reading BMP");
      Serial.println(_pread);
    #endif
    store.writedata(0x0,int32_t(_pread*10));
    state = TEMP;
    break;
  case TEMP://device number 0x1-0x3
    Temp_1 = readTEMP1();
    delay(500);
    Temp_2 = readTEMP2();
    #ifdef DEBUG
      Serial.println("Reading Temp");
      Serial.println(Temp_1);
      Serial.println(Temp_2);
    #endif
    store.writedata(0x1, int32_t(Temp_1*10));
    store.writedata(0x2, int32_t(Temp_2*10));
    state = TIME;
    break;
  case TIME://device number 0xE
    //store.writedata(0xE,0xFFFF); Not Implemented.
    state = EEP;
    break;
  case EEP:
    store.dumpbuf();
    state = WAIT;
    break;
  }
 }
}

float getPressure()
{
  char status;
  double T,P,p0,a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);//need delay to get readings
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
      }
    }
  }
}



float readTEMP1()
{
  byte data[12];
  byte addr[8];

  if ( !ds1.search(addr)) {
      //no more sensors on chain, reset search
      ds1.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds1.reset();
  ds1.select(addr);
  ds1.write(0x44,1); // start conversion, with parasite power on at the end
  
  delay(750); // Wait for temperature conversion to complete

  byte present = ds2.reset();
  ds1.select(addr);    
  ds1.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds2.read();
  }
  
  ds1.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}

float readTEMP2()
{
  byte data[12];
  byte addr[8];

  if ( !ds2.search(addr)) {
      //no more sensors on chain, reset search
      ds2.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds2.reset();
  ds2.select(addr);
  ds2.write(0x44,1); // start conversion, with parasite power on at the end
  
  delay(750); // Wait for temperature conversion to complete

  byte present = ds2.reset();
  ds2.select(addr);    
  ds2.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds2.read();
  }
  
  ds2.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}

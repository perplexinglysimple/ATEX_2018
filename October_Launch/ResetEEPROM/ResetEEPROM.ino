
//This program resets or dumps the output of the eeprom to Serial.

//Define RESET to reset the chip. Comment-out to dump the contents
//#define RESET

#ifndef RESET
#define OUT
#endif

#define MEMORY_SIZE 0x8000 
#include <Wire.h>    
 
#define disk1 0x50    //Address of eeprom chip

#ifdef RESET
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();  
  Serial.print(readEEPROM(disk1, 0), HEX);
  Serial.println();
  unsigned int address = 0;
  writeEEPROM(disk1, 0, 3);
  writeEEPROM(disk1, 1, 0);
  writeEEPROM(disk1, 2, 0);
  Serial.print(readEEPROM(disk1, 0), HEX);
  Serial.print(readEEPROM(disk1, 1), HEX);
  Serial.print(readEEPROM(disk1, 2), HEX);
  Serial.println();

}
#endif

#ifdef OUT
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();  
  Serial.print("Start In");
  delay(1000);
  Serial.print(3);
  delay(1000);
  Serial.print(2);
  delay(1000);
  Serial.print(1);
  Serial.println();
  Serial.print(readEEPROM(disk1, 0), HEX);
  Serial.print(readEEPROM(disk1, 1), HEX);
  Serial.print(readEEPROM(disk1, 2), HEX);
  Serial.println();
  unsigned int address = 0;
  for(size_t i = 3; i < MEMORY_SIZE; i = i)
  {
    byte temp = readEEPROM(disk1, i);
    Serial.print(temp, HEX);
    Serial.print(",");
    if((temp & 0x80) == 0x80){
      Serial.print(readEEPROM(disk1, i + 1), HEX);
      Serial.print(",");
      Serial.print(readEEPROM(disk1, i + 2), HEX);
      i = i + 3;
    }
    else{
      Serial.print(readEEPROM(disk1, i + 1), HEX);
      Serial.print(",");
      Serial.print(readEEPROM(disk1, i + 2), HEX);
      Serial.print(",");
      Serial.print(readEEPROM(disk1, i + 3), HEX);
      Serial.print(",");
      Serial.print(readEEPROM(disk1, i + 4), HEX);
      Serial.println();
      i = i + 5;
    }
  }
  Serial.println();

}
#endif


void loop(){}
 
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}
 
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();
 
  return rdata;
}


//These functions are how to write to the eeprom
void writeEEPROM(int deviceaddress, unsigned int eeaddress, uint8_t data );
byte readEEPROM(int deviceaddress, unsigned int eeaddress );

void writeEEPROM(int deviceaddress, unsigned int eeaddress, uint8_t data ) 
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

//This is the common sturcture used to contain data from a device
struct devicedata
{
    int8_t id;//used to id the device creating this and sees if it is 2 bit or 4 bit
    long data;//the data
};

//This is te class that handles the eeprom
class memorystorage
{
public:

  memorystorage();
  
  bool writedata(int8_t devicenumber, int32_t data);

  bool recycle();

  bool dumpbuf();

private:
  
  bool readstruct(devicedata data, uint16_t pos);

  bool writestruct(devicedata data, uint16_t pos);

  unsigned int thiswrite;

  uint16_t bufferpos;

  bool full;
    
  float longtofloat(int8_t a);

  int32_t last_read [NUM_DEVICES];
};



memorystorage::memorystorage()
{
  for(int i = 0; i < NUM_DEVICES; i++)
  {
	  last_read[i] = (int8_t)0x7FFFFFFF;
  }
  recycle();
}

bool memorystorage::recycle() {
  bufferpos = 0;
  bufferpos = readEEPROM(ADDRESSOFEEPROM, 0 );
  bufferpos = (readEEPROM(ADDRESSOFEEPROM, 1 )<<8) + bufferpos; 
  thiswrite = bufferpos;
  full = readEEPROM(ADDRESSOFEEPROM, 2 );
  return 0;
}

bool memorystorage::dumpbuf() {
  #ifdef Debug
    Serial.begin(38400);
    Serial.print("Finished write at ");
    Serial.println(thiswrite);
  #endif
  writeEEPROM(ADDRESSOFEEPROM, 0, uint8_t(thiswrite&0xFF));
  writeEEPROM(ADDRESSOFEEPROM, 1, uint8_t(thiswrite&0xFF00)>>8);
  return true;
}

bool memorystorage::writedata(int8_t devicenumber, int32_t data) {
  #ifdef Debug
    Serial.begin(38400);
    Serial.print("Writing for device ");
    Serial.print(devicenumber);
    Serial.print(". Writing the data ");
    Serial.println(data);
    Serial.end();
  #endif
  long last_data;
  if(thiswrite + 5 >= MEMORY_SIZE)
    return false;
  byte byteArray[4];
  byteArray[0] = (int)((data >> 24) & 0xFF);
  byteArray[1] = (int)((data >> 16) & 0xFF);
  byteArray[2] = (int)((data >> 8) & 0XFF);
  byteArray[3] = (int)((data & 0XFF));
  int data_diff = data - last_read[devicenumber];
  last_read[devicenumber] = data;
  if(abs(data_diff) > INT16_MAX) {
  	writeEEPROM(ADDRESSOFEEPROM, thiswrite + 1, devicenumber);
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 2, byteArray[0]);
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 3, byteArray[1]);
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 4, byteArray[2]);
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 5, byteArray[3]);
	  thiswrite += 5;
  }
  else {
    #ifdef Debug
      Serial.begin(38400);
      Serial.print("Going into the 3 byte small write. Diff is ");
      Serial.println(data_diff);
      Serial.end();
    #endif
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 1, devicenumber | 0X80);
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 2, (int8_t)((data_diff >> 8) & 0XFF));
	  writeEEPROM(ADDRESSOFEEPROM, thiswrite + 3, (int8_t)(data_diff & 0XFF));
	  thiswrite += 3;
  }
  return true;
}


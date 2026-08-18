#include "EEPROM.h"
#include <string>

#define EEPROM_SIZE 200
#define EEPROM_FH_ADDRESS 0
#define EEPROM_TD_ADDRESS 5
#define EEPROM_MASK_ADDRESS 12

uint32_t lightFixtureHeight;                      // light height
uint32_t triggerDistance;                         // trigger distance from sensor

uint8_t tmf8829ZoneMask[] = {
                              0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 
                              0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 
                              0, 0, 0, 0, 0, 0, 0, 0,      
                              0, 0, 0, 0, 0, 0, 0, 0, 
                              0, 0, 0, 0, 0, 0, 0, 0             
                            };    // 8x8 zone mask 0 = clear, 1 = masked

void setup() 
{
  Serial.begin(115200);
  Serial.println("ESP32 is ready. Please enter a message:");

  // setup EEPROM and retriebe storaged values
  EEPROM.begin(EEPROM_SIZE);

  EEPROM.writeUInt(EEPROM_FH_ADDRESS, 2000);
  EEPROM.commit();

  EEPROM.writeUInt(EEPROM_TD_ADDRESS, 1000);
  EEPROM.commit();
}

void loop() 
{
  Serial.println("Reading back values");
  Serial.println(EEPROM.readUInt(EEPROM_FH_ADDRESS));
  Serial.println(EEPROM.readUInt(EEPROM_TD_ADDRESS));
}

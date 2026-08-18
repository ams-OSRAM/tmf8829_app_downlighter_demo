/**************************************************************************************************
* Copyright © 2024 ams-OSRAM AG                                                                   *
* All rights are reserved.                                                                        *
*                                                                                                 *
* FOR FULL LICENSE TEXT SEE LICENSES-MIT.TXT                                                      *
*                                                                                                 *
**************************************************************************************************/

/* tmf8829 arduino uno sample program modified for downlighter demo example*/

// ---------------------------------------------- includes ----------------------------------------
#include <WiFi.h>
#include <WebServer.h>
#include "tmf8829_shim.h"
#include "tmf8829.h"
#include "tmf8829_app.h"
#include "tmf8829_image.h"
#include "EEPROM.h"
#include <string>
WebServer server(80);

//#include <NetworkClient.h>
//#include <WiFiAP.h>

// ---------------------------------------------- defines -----------------------------------------
#define NR_OF_MEAS_CFGS 9 // number of preconfiguration commads that are available, see TMF8829_CMD_STAT 

// tmf application states
#define TMF8829_STATE_DISABLED      0
#define TMF8829_STATE_STANDBY       1     
#define TMF8829_STATE_STOPPED       2
#define TMF8829_STATE_MEASURE       3
#define TMF8829_STATE_ERROR         4

#define NR_LOG_LEVELS               9 // number of log-levels in array

// number of register that are printed in the dump on one line
#define NR_REGS_PER_LINE            8

// maximum binary command payload size
#define TMF8829_BINARY_BUF_SIZE     ( TMF8829_CFG_PAGE_SIZE + 5 ) //maximum size and 5 spare bytes

// binary command identifiers
#define TMF8829_BINARY_CMD_CONFIGURE      0x31  // sets arbitrary configuration
#define TMF8829_BINARY_CMD_PRE_CONFIGURE  0x32  // sets pre configuration
#define TMF8829_BINARY_CMD_CHAR_MODE      0x00  // not a valid command identifier, indicates that the application is in character input mode
#define TMF8829_BINARY_CMD_PENDING        0xFF  // not a valid command identifier, indicates that the application is in binary input mode awaiting a command identifier

// application settings
#define LIGHT_OFF     0x00
#define OUTER_RING    0x01
#define MIDDLE_RING   0x02
#define INNER_RING    0x03

#define innerRingCnt  4
#define middleRingCnt 32
#define outerRingCnt  28

#define MASKED        0x0000                    // used to mask zones when blocked
#define UN_MASKED     0xFFFF                    // used to indicate clear zones for sensing

#define EEPROM_SIZE 200
#define EEPROM_FH_ADDRESS 0
#define EEPROM_TD_ADDRESS 5
#define EEPROM_MASK_ADDRESS 12

// ---------------------------------------------- constants -----------------------------------------
// to increase/decrease logging
const uint8_t logLevels[ NR_LOG_LEVELS ] = 
{ TMF8829_LOG_LEVEL_NONE
, TMF8829_LOG_LEVEL_ERROR
, TMF8829_LOG_LEVEL_RESULTS_HEADER
, TMF8829_LOG_LEVEL_RESULTS
, TMF8829_LOG_LEVEL_CLK_CORRECTION
, TMF8829_LOG_LEVEL_INFO
, TMF8829_LOG_LEVEL_VERBOSE
, TMF8829_LOG_LEVEL_I2C
, TMF8829_LOG_LEVEL_DEBUG
};

const int measCfg[NR_OF_MEAS_CFGS] = 
{
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8, 
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_LONG_RANGE,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_HIGH_ACCURACY,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16_HIGH_ACCURACY,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32_HIGH_ACCURACY,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32,
  TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32_HIGH_ACCURACY
};

const int outputPin = D7;        // D7 for LED / PWM output
const int greenLed = D8;

const uint8_t INNERRING_1[] = {28, 29, 36, 37};
const uint8_t MIDDLERING_1[] = {10, 11, 12, 13, 14, 15, 18, 19, 20, 21, 22, 23, 26, 27, 30, 31, 34, 35, 38, 39, 42, 43, 44, 45, 46, 47, 50, 51, 52, 53, 54, 55};
const uint8_t OUTERRING_1[]  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 16, 17, 24, 25, 32, 33, 40, 41, 48, 49, 56, 57, 58, 59, 60, 61, 62, 63, 64};

const char *ssid = "TMF8829_Downlighter";
const char *password = "TTMMFF88882299";

// ---------------------------------------------- variables -----------------------------------------

tmf8829Driver tmf8829;            // instances of tmf8829
uint8_t logLevel;                 // how chatty the program is 
int8_t stateTmf8829;              // current state of the device
int8_t configNr;                  // this sample application has only a few configurations it will loop through, the variable keeps track of that 
int8_t clkCorrectionOn;           // if non-zero clock correction is on
volatile uint8_t irqTriggered;    // interrupt is triggered or not
uint8_t binaryCmd;                // currently active binary command identifier (if any)
uint8_t binaryBufFill;            // fill level of the binary command payload buffer
uint8_t binaryBuf[TMF8829_BINARY_BUF_SIZE]; // binary command payload buffer
uint8_t zoneTriggered;
uint8_t lastZone;
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

uint16_t innerRing[4][4];
uint16_t middleRing[32][32];
uint16_t outerRing[28][28];

uint16_t tmf8829Res[64];                          // raw 8x8 results
uint16_t tmf8829ResMasked[64];                    // masked zones
uint32_t lightFixtureHeight;                      // light height
uint32_t triggerDistance;                         // trigger distance from sensor
uint8_t zoneMaskCntr = 0;

uint8_t printVal;
uint8_t outputTestCnt;
int8_t address = 0;

String json = {};

// HTML page
String htmlPage() 
{
String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>TMF8829 Downlighter Demo</title>
<meta charset="UTF-8">

<style>
html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
body { font-family: Arial, sans-serif; background: #C0C0C0;}

.button-container { display: flex; justify-content: center; gap: 15px; margin-top: 20px; flex-wrap: wrap; }
.button { background-color: #0000FF; border: none; color: #0000FF; padding: 5px 15px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer; }

.buttonStop { background-color: lightgray; color: black; border: 2px solid red; }
.buttonStop:hover { background-color: red; color: white; }

.buttonStart { background-color: lightgray; color: black; border: 2px solid green; }
.buttonStart:hover { background-color: green; color: white; }

.buttonFixMin { background-color: lightgray; color: black; border: 2px solid red; }
.buttonFixMin:hover { background-color: red; color: white; }

.buttonFixPlu { background-color: lightgray; color: black; border: 2px solid green; }
.buttonFixPlu:hover { background-color: green; color: white; }

.buttonTriMin { background-color: lightgray; color: black; border: 2px solid red; }
.buttonTriMin:hover { background-color: red; color: white; }

.buttonTriPlu { background-color: lightgray; color: black; border: 2px solid green; }
.buttonTriPlu:hover { background-color: green; color: white; }

.buttonOutput { background-color: #FFFFE0; color: black; border: 2px solid yellow; }
.buttonOutput:hover { background-color: yellow; color: white; }

.buttonClrMsk { background-color: #FFFFE0; color: black; border: 2px solid yellow; }
.buttonClrMsk:hover { background-color: yellow; color: white; }

.data-table1 { background-color: lightgray; border-collapse: collapse; margin: 10px auto 30px auto; table-layout: fixed; width: 50%; }
.data-table1 td { border: 1px solid #333; padding: 8px; text-align: center; width: 50px; height: 20px; }

.data-table2 { background-color: #e3f2fd; border-collapse: collapse; margin: 10px auto 30px auto; table-layout: fixed; width: 50%; }
.data-table2 td { border: 1px solid #333; padding: 8px; text-align: center; width: 50px; height: 20px; }

.data-table3 { background-color: lightgray; border-collapse: collapse; margin: 10px auto 30px auto; table-layout: fixed; width: 50%; }
.data-table3 td { border: 1px solid #333; padding: 8px; text-align: center; width: 50px; height: 20px; }

/* Special coloring for table1 */
.data-table1 tr:nth-child(1), .data-table1 td:nth-child(1), .data-table1 tr:nth-child(8), .data-table1 td:nth-child(8) 
{
  background-color: #69D4FF;
}
.data-table1 tr:nth-child(4) td:nth-child(4), .data-table1 tr:nth-child(4) td:nth-child(5), .data-table1 tr:nth-child(5) td:nth-child(4), .data-table1 tr:nth-child(5) td:nth-child(5) 
{
  background-color: #72F67F;
}

.sensor2-on  { background-color: #ff5252 !important; color: #fff; } /* red */

.table2 td:hover { filter: brightness(1.05); cursor: pointer; }

.table-title { text-align: center; font-weight: bold; font-size: 1.2em; margin-top: 20px; }
</style>
</head>
<body>
<h1 style="text-align:center;">TMF8829 Downlighter Configuration Menu</h1>

<div class="button-container">
<form action="/stop"><button class="button buttonStop">Stop</button></form>
<form action="/start"><button class="button buttonStart">Start</button></form>
</div>

<h2 style="text-align:center;">Fixture height &nbsp; &nbsp; &nbsp; &nbsp; Trigger height</h2>
<div class="values">
<h2><span id="val1"></span> mm &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <span id="val2"></span> mm</h2>
</div>

<div class="button-container">
<form action="/buttonFixMin"><button class="button buttonFixMin">Fixture-</button></form>
<form action="/buttonFixPlu"><button class="button buttonFixPlu">Fixture+</button></form>
<form action="/buttonTriMin"><button class="button buttonTriMin">Trigger-</button></form>
<form action="/buttonTriPlu"><button class="button buttonTriPlu">Trigger+</button></form>
</div>

<div class="button-container">
<form action="/buttonOutput"><button class="button buttonOutput">Output Test Mode</button></form>
<form action="/buttonClrMsk"><button class="button buttonClrMsk">Reset Mask</button></form>
</div>

<div class="data-table1">
  <h3> </h3>
  <h3>8x8 raw distance from light fixture</h3>
  <table id="data-table1"></table>
</div>
<div class="data-table2">
  <h3>8x8 mask settings</h3>
  <table id="data-table2"></table>
</div>
<div class="data-table3">
  <h3>8x8 masked results</h3>
  <table id="data-table3"></table>
</div>
      
<script>
        // Persist selection state by row-col key (e.g., "3-5")
        const selectedCells = {}; // { "r-c": true }

        function updateTables() {
          fetch('/data')
            .then(response => response.json())
            .then(data => {
              // Show extra 16-bit values
              document.getElementById('val1').textContent = data.val1;
              document.getElementById('val2').textContent = data.val2;

              let t1 = '';
              let t2 = '';
              let t3 = '';

              for (let i = 0; i < 8; i++) {
                t1 += '<tr>';
                t2 += '<tr>';
                t3 += '<tr>';
                for (let j = 0; j < 8; j++) {
                  // Sensor 1: plain values
                  t1 += `<td>${data.s1[i][j]}</td>`;

                  // Sensor 2: color based on sensor value (1 = red, 0 = default)
                  const v2 = Number(data.s2[i][j]); // expect 0 or 1
                  const baseClass = (v2 === 1) ? 'sensor2-on' : 'sensor2-off';
                  const key = `${i}-${j}`;
                  const selClass = selectedCells[key] ? 'selected' : '';
                  // Include both base class and optional selected class
                  t2 += `<td class="${baseClass} ${selClass}" onclick="cellClicked(${i},${j},${v2}, this)">${v2}</td>`;

                  // Sensor 3: plain values
                  t3 += `<td>${data.s3[i][j]}</td>`;
                }
                t1 += '</tr>';
                t2 += '</tr>';
                t3 += '</tr>';
              }

              document.getElementById('data-table1').innerHTML = t1;
              document.getElementById('data-table2').innerHTML = t2;
              document.getElementById('data-table3').innerHTML = t3;
            })
            .catch(err => console.error('Fetch /data error:', err));
        }

        // Click toggles selection (outline) and sends coords/value to ESP32
        function cellClicked(row, col, value, cell) 
        {
          const key = `${row}-${col}`;

          // Toggle selection state and outline class
          if (selectedCells[key]) 
          {
            delete selectedCells[key];
            cell.classList.remove('selected');
          } else 
          {
            selectedCells[key] = true;
            cell.classList.add('selected');
          }

          // Send clicked cell info back to ESP32
          fetch(`/clicked?row=${row}&col=${col}&value=${value}`)
            .then(r => r.text())
            .then(msg => console.log('ESP32 response:', msg))
            .catch(err => console.error('Fetch /clicked error:', err));
        }

        // Refresh every second + initial load
        setInterval(updateTables, 1000);
        updateTables();
      </script>
</body>
</html>
)rawliteral";
  return page;
}

// ---------------------------------------------- function declaration ------------------------------
void static printDeviceInfo();
void static printHelp();
void static printRegisters( uint8_t regAddr, uint16_t len, char seperator );
void static resetAppState();

// ---------------------------------------------- functions  ----------------------------------------

/******************************************************************************/
/* TMF8829 Device Functions                                                   */
/******************************************************************************/

// Preconfiguration  eq.: TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8
void preconfigure ( int8_t cfgNr )
{
  int8_t stat = tmf8829Command(&tmf8829, cfgNr);
  
  if ( stat == APP_SUCCESS_OK ) 
  {
    PRINT_CONST_STR( F( "Preconfig " ) );
    PRINT_INT( cfgNr );
    PRINT_CHAR( SEPARATOR );
  }
  else
  {
    PRINT_CONST_STR( F(  "#Err" ) );
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR( F(  "Config" ) );
  }
  PRINT_LN( );
}

// get configuration of tmf8829 and print the configuration
void getConfiguration ( )
{
  if ( stateTmf8829 == TMF8829_STATE_STOPPED )
  {
    tmf8829GetConfiguration( &tmf8829 );
    PRINT_CONST_STR( F(  "#Config " ) );
    for ( int i = 0; i < TMF8829_CFG_PAGE_SIZE; i++ )
    {
      PRINT_UINT_HEX(tmf8829.config[i]);
      PRINT_CHAR( SEPARATOR );
    }
    PRINT_LN( );
  }
}

// set configuration of tmf8829
void setConfiguration ( )
{
  if ( stateTmf8829 == TMF8829_STATE_STOPPED )
  {
    tmf8829.config[0] = 0xE8;                 // set measurement period to 1 second
    tmf8829.config[1] = 0x03;
    tmf8829.config[2] = 0xA0;                 // set iterations to 4000k
    tmf8829.config[3] = 0x0F;
    tmf8829SetConfiguration( &tmf8829 );
  }
}

// enable device, download firmware and start Ram App
void enable ( uint32_t imageStartAddress, const unsigned char * image, int32_t imageSizeInBytes )
{
  int8_t status;

  if ( stateTmf8829 == TMF8829_STATE_DISABLED )
  {
    tmf8829Enable( &tmf8829 );
    delayInMicroseconds( ENABLE_TIME_MS * 1000);
    tmf8829ClkCorrection( &tmf8829, clkCorrectionOn ); 
    tmf8829SetLogLevel( &tmf8829, logLevels[ logLevel ] );
    tmf8829PowerUp( &tmf8829 );
    if ( tmf8829IsCpuReady( &tmf8829, CPU_READY_TIME_MS) )
    {
      PRINT_CONST_STR( F( " CPU ready" ) );

      tmf8829BootloaderCmdSpiOff(&tmf8829); //i2c is used

      PRINT_CONST_STR( F( " DWNL FW and start App" ) );
      status = tmf8829DownloadFirmware( &tmf8829, imageStartAddress, image, imageSizeInBytes, 1 /* over fifo */ );
      PRINT_LN( );
      
      if ( status == BL_SUCCESS_OK ) 
      {
        resetAppState();
        tmf8829GetConfiguration(&tmf8829);
        stateTmf8829 = TMF8829_STATE_STOPPED;
        tmf8829ReadDeviceInfo( &tmf8829 );
        printDeviceInfo( );
      }
      else
      {
        stateTmf8829 = TMF8829_STATE_ERROR;
      }
    }
    else
    {
      stateTmf8829 = TMF8829_STATE_ERROR;
    }
  } // else device is already enabled
  else
  {
    tmf8829ReadDeviceInfo( &tmf8829 );
    printDeviceInfo( );
  }
}

// start measurement
void measure ( )
{
  if ( stateTmf8829 == TMF8829_STATE_STOPPED )
  {
    tmf8829ClrAndEnableInterrupts( &tmf8829, TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_HISTOGRAMS );
    tmf8829StartMeasurement( &tmf8829 );
    stateTmf8829 = TMF8829_STATE_MEASURE;
  }
  else
  {
    PRINT_CONST_STR( F(  "no start of measurement, wrong state" ) );
    PRINT_LN( );
  }
}

// execute a stop measurement
void stop ( )
{
  if ( stateTmf8829 == TMF8829_STATE_MEASURE || stateTmf8829 == TMF8829_STATE_STOPPED )
  {
    tmf8829StopMeasurement( &tmf8829 );
    tmf8829DisableInterrupts( &tmf8829, 0xFF );               // just disable all
    stateTmf8829 = TMF8829_STATE_STOPPED;
  }
}

// power down by setting PON=0 bit
void powerDown ( )
{
  if ( stateTmf8829 == TMF8829_STATE_MEASURE )      // stop a measurement first
  {
    tmf8829StopMeasurement( &tmf8829 );
    tmf8829DisableInterrupts( &tmf8829, 0xFF );     // just disable all
    stateTmf8829 = TMF8829_STATE_STOPPED;
  }
  if ( stateTmf8829 == TMF8829_STATE_STOPPED )
  {
    tmf8829Standby( &tmf8829 );
    stateTmf8829 = TMF8829_STATE_STANDBY;
  }
}

// perform a hardware + software reset
void reset ( )
{
  if ( stateTmf8829 != TMF8829_STATE_DISABLED )
  {
    tmf8829Reset( &tmf8829 );
    PRINT_CONST_STR( F(  "Reset TMF8829" ) );
    PRINT_LN( );
    stateTmf8829 = TMF8829_STATE_STOPPED;
  }
}

// wakeup sequence
void wakeup ( )
{
  if ( stateTmf8829 == TMF8829_STATE_STANDBY )
  {
    tmf8829Wakeup( &tmf8829 );
    if ( tmf8829IsCpuReady( &tmf8829, CPU_READY_TIME_MS ) )
    {
      stateTmf8829 = TMF8829_STATE_STOPPED;
    }
    else
    {
      stateTmf8829 = TMF8829_STATE_ERROR;
    }
  }
}

/******************************************************************************/
/* Application Functions                                                      */
/******************************************************************************/

// print mask
void printZoneMask ( )
{
  Serial.println();
  for (int row = 0; row < 8; row++) 
  {
    for (int col = 0; col < 8; col++) 
    {
      Serial.print(tmf8829ZoneMask[row * 8 + col]); 
      Serial.print(" ");
    }
    Serial.println(); // Move to the next row
  }
  Serial.println();
}

// read current zone mask
void zoneMaskRead ( )
{
  zoneMaskCntr = 0;
  printZoneMask( );
}

// set zone mask
void zoneMaskSet ( )
{
  if (zoneMaskCntr == 64)
  {
    zoneMaskCntr = 0;
  }

  Serial.print("Zone? ");
  Serial.print(zoneMaskCntr);
  Serial.println();
  zoneMaskCntr++;
}

// toggle zone mask
void zoneMaskToggle ( )
{
  if (tmf8829ZoneMask[zoneMaskCntr-1] == 0)
  {
    tmf8829ZoneMask[zoneMaskCntr-1] = 1;
  }
  else 
  {
    tmf8829ZoneMask[zoneMaskCntr-1] = 0;
  }
  printZoneMask( );
}

// mask all zones
void maskAllZones ( )
{
  for (int i = 0; i < 64; i++) 
  {
    tmf8829ZoneMask[i] = 1;
  }
  printZoneMask( );

  for (int i = 0 ; i < 64 ; i++)
  {
    EEPROM.write(EEPROM_TD_ADDRESS + i, 1);
    EEPROM.commit();
  }
}

// unmask all zones
void unMaskAllZones ( )
{
  for (int i = 0; i < 64; i++) 
  {
    tmf8829ZoneMask[i] = 0;
  }
  printZoneMask( );

  for (int i = 0 ; i < 64 ; i++)
  {
    EEPROM.write(EEPROM_TD_ADDRESS + i, 0);
    EEPROM.commit();
  }
}

// increase trigger distance
void triggerDistanceInc ( )
{
  triggerDistance = triggerDistance + 250;
  if (triggerDistance >= 5000)
  {
    triggerDistance = 5000;
  }

  if (triggerDistance > lightFixtureHeight)
  {
    triggerDistance = lightFixtureHeight - 250;
  }

  PRINT_CONST_STR( F(  "Trigger distance = " ) );
  PRINT_INT( triggerDistance );
  PRINT_LN( );

  EEPROM.write(EEPROM_TD_ADDRESS, triggerDistance);
  EEPROM.commit();
}

// decrease trigger distance
void triggerDistanceDec ( )
{
  triggerDistance = triggerDistance - 250;
  if (triggerDistance <= 250)
  {
    triggerDistance = 250;
  }
  PRINT_CONST_STR( F(  "Trigger distance = " ) );
  PRINT_INT( triggerDistance );
  PRINT_LN( );

  EEPROM.writeUInt(EEPROM_TD_ADDRESS, triggerDistance);
  EEPROM.commit();
}

// set light fixture height 
void fixtureHeightInc ( )
{
  lightFixtureHeight = lightFixtureHeight + 250;
  if (lightFixtureHeight > 5000)
  {
    lightFixtureHeight = 5000;
  }
  PRINT_CONST_STR( F(  "Fixture height = " ) );
  PRINT_INT( lightFixtureHeight );
  PRINT_LN( );

  EEPROM.writeUInt(EEPROM_FH_ADDRESS, lightFixtureHeight);
  EEPROM.commit();
}

void fixtureHeightDec ( )
{
  lightFixtureHeight = lightFixtureHeight - 250;

  if (lightFixtureHeight < 500)
  {
    lightFixtureHeight = 500;
  }

  if (lightFixtureHeight < triggerDistance)
  {
    lightFixtureHeight = triggerDistance + 250;
  }
  PRINT_CONST_STR( F(  "Fixture height = " ) );
  PRINT_INT( lightFixtureHeight );
  PRINT_LN( );

  EEPROM.writeUInt(EEPROM_FH_ADDRESS, lightFixtureHeight);
  EEPROM.commit();
}

// scroll through output modes
void outputTest( )
{
  if (outputTestCnt == INNER_RING)
  {
    pinMode(outputPin, OUTPUT);
    digitalWrite(outputPin, HIGH);
  }
  
  if (outputTestCnt == MIDDLE_RING)
  {
    ledcAttach(outputPin, 1000, 8);
    ledcChangeFrequency(outputPin, 300, 14);
    analogWrite(outputPin,8192);
  }

  if (outputTestCnt == OUTER_RING)
  {

    ledcAttach(outputPin, 3, 14);
    ledcChangeFrequency(outputPin, 3, 14);
    analogWrite(outputPin,8192);
  }
    
  if (outputTestCnt == LIGHT_OFF)
  {
    pinMode(outputPin, OUTPUT);
    digitalWrite(outputPin, LOW);
  }
  outputTestCnt++;
  if (outputTestCnt == 4)
  {
    outputTestCnt = 0;
  }
}

void printDeviceInfo ( )
{
  PRINT_CONST_STR( F(  "TMF8829 Arduino Driver Version " ) );
  PRINT_INT( tmf8829.info.version[0] ); PRINT_CHAR( '.' );
  PRINT_INT( tmf8829.info.version[1] ); 
  PRINT_LN( );
  PRINT_CONST_STR( F(  "Firmware Application Version " ) );
  PRINT_INT( tmf8829.device.appVersion[0] ); PRINT_CHAR( '.' );
  PRINT_INT( tmf8829.device.appVersion[1] ); PRINT_CHAR( '.' );
  PRINT_INT( tmf8829.device.appVersion[2] ); PRINT_CHAR( '.' );
  PRINT_INT( tmf8829.device.appVersion[3] ); PRINT_CHAR( '.' );
  PRINT_LN( );
  PRINT_CONST_STR( F(  "Chip Version " ) );
  PRINT_INT( tmf8829.device.chipVersion[0] ); PRINT_CHAR( '.' );
  PRINT_INT( tmf8829.device.chipVersion[1] ); 
  PRINT_LN( );
  PRINT_CONST_STR( F(  "Serial Number 0x" ) );
  PRINT_UINT_HEX( tmf8829.device.deviceSerialNumber );
  PRINT_LN( );
}

// Print the current state (stateTmf8829) in a readable format
void printState ( )
{
  PRINT_CONST_STR( F(  " state=" ) );
  switch ( stateTmf8829 )
  {
    case TMF8829_STATE_DISABLED: PRINT_CONST_STR( F(  "disabled" ) ); break;
    case TMF8829_STATE_STANDBY: PRINT_CONST_STR( F(  "standby" ) ); break;
    case TMF8829_STATE_STOPPED: PRINT_CONST_STR( F(  "stopped" ) ); break;
    case TMF8829_STATE_MEASURE: PRINT_CONST_STR( F(  "measure" ) ); break;
    case TMF8829_STATE_ERROR: PRINT_CONST_STR( F(  "error" ) ); break;   
    default: PRINT_CONST_STR( F(  "???" ) ); break;
  }
  PRINT_LN( );
}

/******************************************************************************/
/* Character Input Functions                                                  */
/******************************************************************************/
#define DELAY_PRINT_HELP  10000
// Function prints a help screen
void printHelp ( )
{
  PRINT_LN( ); PRINT_CONST_STR( F(  "m / M ... measure" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "s / S ... stop" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "f / F ... fixture height + 25cm, (max 500cm)" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "l / L ... fixture height - 25cm, (max 500cm)" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "i / I ... trigger distance + 25cm (max 500cm)" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "d / D ... trigger distance - 25cm (min 25cm)" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "a / A ... mask all zones" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "u / U ... un-mask all zones" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "y / Y ... read zone mask" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "z / Z ... zone mask increment" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "w / W ... zone mask set" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); PRINT_CONST_STR( F(  "o / O ... output test" ) );
  delayInMicroseconds( DELAY_PRINT_HELP );
  PRINT_LN( ); 
}

// handles a single incoming character in character input mode, returns 1 if program termination is requested
int8_t handleCharInput ( char key )
{
  if ( key < 33 || key >= 126 ) // skip all control characters and DEL  
  {
    return 0; // nothing to do here
  }

  if ( key == 'h' | key == 'H')
  {
    printHelp(); 
  }
  else if ( key == 'c' | key == 'C')       // get configuration
  {
    getConfiguration( );
    setConfiguration ( );
  }
  else if ( key == 'a' | key == 'A')       // mask all zones
  {
   maskAllZones ( );
  }
  else if ( key == 'u' | key == 'U')       // un-mask all zones
  {
   unMaskAllZones ( );
  }
  else if ( key == 'm' | key == 'M')
  {  
    measure( );
  }
  else if ( key == 's' | key == 'S')
  {
    stop( );
  }
  else if ( key == 'z' | key == 'Z')
  {
    zoneMaskSet( );
  }
  else if ( key == 'w' | key == 'W')
  {
    zoneMaskToggle( );
  }
  else if ( key == 'y' | key == 'Y')
  {
    zoneMaskRead( );
  }
  else if ( key == 'i' | key == 'I') // increase trigger distance
  {
    triggerDistanceInc( );
  }
  else if ( key == 'd' | key == 'D') // decrease trigger distance
  {
    triggerDistanceDec( );
  }
  else if ( key == 'f' | key == 'F') // increase fixture height
  {
    fixtureHeightInc( );
  }
  else if ( key == 'l' | key == 'L') // decrease fixture height
  {
    fixtureHeightDec( );
  }
  else if ( key == 'o' | key == 'O') // scroll through output modes
  {
    outputTest( );
  }
  else if ( key == 'q' ) // terminate on device where this can be done
  {
    return 0; // terminate if possible
  }
  else 
  {
    PRINT_CONST_STR( F(  "#Err" ) );
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR( F(  "Cmd " ) );
    PRINT_CHAR( key );
    PRINT_LN( );
  }
  return 0;
}

// Function checks the UART for received characters and interprets them, returns 1 if program termination is requested
int8_t serialInput ( )
{
  char rx;
  int8_t read = inputGetKey( &rx );
  while ( read ) {
    int8_t res;

    res = handleCharInput( rx );

    if ( res != 0 ) {
      return res;
    }

    read = inputGetKey( &rx );
  }
  return 0;     // rx must be 0 to leave while loop
}

// mask out unused zones
void maskZones( )
{
  uint8_t cnt;
  for (cnt = 0 ; cnt < 64 ; cnt++)
  {
    if (tmf8829ZoneMask[cnt] == 1)                      // 1 = masked
    {
      tmf8829ResMasked[cnt] = lightFixtureHeight;       // masked always report distance = 0
    }
    if (tmf8829ZoneMask[cnt] == 0)                      // 0 - un-masked
    {
      if (tmf8829ResMasked[cnt] > lightFixtureHeight)
      {
        tmf8829ResMasked[cnt] = lightFixtureHeight;     // un-masked & > fixture height report fixture height
      }
      else 
      {
        tmf8829ResMasked[cnt] = tmf8829Res[cnt];        // un-masked & < fixture height report actual distance1
      }
    }
  }

  Serial.print("msk#");
  Serial.println('\t');

  for (int row = 0; row < 8; row++) 
  {
    for (int col = 0; col < 8; col++) 
    {
      Serial.print(tmf8829ResMasked[row * 8 + col]); 
      Serial.print(" ");
    }
    Serial.println(); // Move to the next row
  }
  Serial.println();
}

// presence detection function, returns zone number
int8_t presenceDetection( )
{
  for ( int i = 0 ; i < innerRingCnt ; i++)
  {
    innerRing[0][i] = tmf8829ResMasked[INNERRING_1[i]];
    if (innerRing[0][i] < triggerDistance & innerRing[0][i] != 0)
    {
      return INNER_RING;
    }
  }

  for ( int i = 0 ; i < middleRingCnt ; i++)
  {
    middleRing[0][i] = tmf8829ResMasked[MIDDLERING_1[i]];
    if (middleRing[0][i] < triggerDistance & middleRing[0][i] != 0)
    {
      return MIDDLE_RING;
    }
  }

  for ( int i = 0 ; i < outerRingCnt ; i++)
  {
    outerRing[0][i] = tmf8829ResMasked[OUTERRING_1[i]];
    if (outerRing[0][i] < triggerDistance & outerRing[0][i] != 0)
    {
      return OUTER_RING;
    }
  }

  return LIGHT_OFF;
}

// checks TMF8829 results, masks unused zones and detects motion
void updateOutput( )
{
  maskZones( );
  int8_t i;
  
  zoneTriggered = presenceDetection( );

  if (zoneTriggered == INNER_RING)
  {
    pinMode(outputPin, OUTPUT);
    digitalWrite(outputPin, HIGH);
    return;
  }
  
  if (zoneTriggered == MIDDLE_RING)
  {
    ledcAttach(outputPin, 1000, 8);
    ledcChangeFrequency(outputPin, 300, 14);
    analogWrite(outputPin,8192);
    return;
  }

  if (zoneTriggered == OUTER_RING)
  {

    ledcAttach(outputPin, 3, 14);
    ledcChangeFrequency(outputPin, 3, 14);
    analogWrite(outputPin,8192);
    return;
  }
    
  if (zoneTriggered == LIGHT_OFF)
  {
    pinMode(outputPin, OUTPUT);
    digitalWrite(outputPin, LOW);
    return;
  }
}

// ====== Handle root page ======
void handleRoot() 
{
  server.send(200, "text/html", htmlPage());
}

// ====== Handle buttons from HTML ======
void handleButton(int btn) 
{
  if (btn == 1) 
  {
    handleCharInput('S');
  }
  if (btn == 2) 
  {
    handleCharInput('M');
  }
  if (btn == 3) 
  {
    handleCharInput('L');
  }
  if (btn == 4) 
  {
    handleCharInput('F');
  }
  if (btn == 5) 
  {
    handleCharInput('D');
  }
  if (btn == 6)
  {
    handleCharInput('I');
  }
  if (btn == 7) 
  {
    handleCharInput('O');
  }
  if (btn == 8)
  {
    handleCharInput('U');
  }

  server.send(200, "text/html", htmlPage());
}

// ====== Format JSON data ======
void handleData() 
{
  int s1[8][8], s2[8][8], s3[8][8];
  for (int i = 0; i < 8; i++) 
  {
    for (int j = 0; j < 8; j++) 
    {
      s1[i][j] = tmf8829Res[(i*8)+j];
      s2[i][j] = tmf8829ZoneMask[(i*8)+j];
      s3[i][j] = tmf8829ResMasked[(i*8)+j];
    }
  }

  // Build JSON

  String json = "{\"val1\":" + String(lightFixtureHeight) + ",\"val2\":" + String(triggerDistance) + ",\"s1\":[";
  for (int i = 0; i < 8; i++) 
  {
    json += "[";
    for (int j = 0; j < 8; j++) 
    {
      json += String(s1[i][j]);
      if (j < 7) json += ",";
    }
    json += "]";
    if (i < 7) json += ",";
  }
  json += "],\"s2\":[";
  for (int i = 0; i < 8; i++) 
  {
    json += "[";
    for (int j = 0; j < 8; j++) 
    {
      json += String(s2[i][j]);
      if (j < 7) json += ",";
    }
    json += "]";
    if (i < 7) json += ",";
  }
  json += "],\"s3\":[";
  for (int i = 0; i < 8; i++) 
  {
    json += "[";
    for (int j = 0; j < 8; j++) 
    {
      json += String(s3[i][j]);
      if (j < 7) json += ",";
    }
    json += "]";
    if (i < 7) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleMaskClick() 
{
  int maskInt;
  char rowChar[2];
  char colChar[2];
  String row = server.arg("row");
  String col = server.arg("col");
  row.toCharArray(rowChar,2);
  col.toCharArray(colChar,2);
  maskInt = (atoi(rowChar) * 8) + atoi(colChar);
  
  if (tmf8829ZoneMask[maskInt] == 0)
  {
    tmf8829ZoneMask[maskInt] = 1;
    EEPROM.write(EEPROM_MASK_ADDRESS + maskInt, 1);
    EEPROM.commit(); 
  }
  else 
  {
    tmf8829ZoneMask[maskInt] = 0;
    EEPROM.write(EEPROM_MASK_ADDRESS + maskInt, 0);
    EEPROM.commit(); 
  }
  String value = server.arg("value");
  server.send(200, "text/plain", "Received: " + value);
}

/******************************************************************************/
/* Arduino Setup and Loop Functions                                           */
/******************************************************************************/

// resets the status of the set application
void resetAppState ( )
{
  stateTmf8829 = TMF8829_STATE_DISABLED;
  configNr = NR_OF_MEAS_CFGS;               // reset of preconfigure Nr.
  clkCorrectionOn = 1;
  irqTriggered = 0;
}

// interrupt handler is called when INT pin goes low
void interruptHandler ( void )
{
  irqTriggered = 1;
}

//-------------------------------------------------------------------------------------------------------------

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setupFn( uint8_t logLevelIdx, uint32_t baudrate, uint32_t i2cClockSpeedInHz )
{
  // start serial and i2c
  inputOpen( baudrate );
  i2cOpen( &tmf8829, i2cClockSpeedInHz );

  // Start AP mode
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // WiFi response actions
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/clicked", handleMaskClick);
  server.on("/stop", [](){ handleButton(1); });
  server.on("/start", [](){ handleButton(2); });
  server.on("/buttonFixMin", [](){ handleButton(3); });
  server.on("/buttonFixPlu", [](){ handleButton(4); });
  server.on("/buttonTriMin", [](){ handleButton(5); });
  server.on("/buttonTriPlu", [](){ handleButton(6); });
  server.on("/buttonOutput", [](){ handleButton(7); });
  server.on("/buttonClrMsk", [](){ handleButton(8); });

  server.begin();

  // configure TMF8829 device
  logLevel = logLevelIdx;
  configurePins( &tmf8829 );
  resetAppState( );
  tmf8829Initialise( &tmf8829 );
  tmf8829SetLogLevel( &tmf8829, logLevels[ logLevelIdx ] );
  setInterruptHandler( interruptHandler );
  tmf8829Disable( &tmf8829 );                                     // this resets the I2C address in the device
  delayInMicroseconds(CAP_DISCHARGE_TIME_MS * 1000);              // wait for a proper discharge of the cap
  enable( tmf8829_image_start, tmf8829_image, tmf8829_image_length );
  preconfigure( measCfg[1] );                                     // pre-configue device to 8x8 long range
  getConfiguration ( );
  setConfiguration ( );

  // setup EEPROM and retriebe storaged values
  EEPROM.begin(EEPROM_SIZE);
  lightFixtureHeight = EEPROM.readUInt(EEPROM_FH_ADDRESS);
  triggerDistance = EEPROM.readUInt(EEPROM_TD_ADDRESS);
    
  for (int i = 0; i < 64; i++)
  {
    if (EEPROM.read(EEPROM_MASK_ADDRESS + i) == 1)
    {
      tmf8829ZoneMask[i] = 1;
    }
    else 
    {
      tmf8829ZoneMask[i] = 0;
    }
  }
  pinMode(greenLed, OUTPUT);
  digitalWrite(greenLed, HIGH);

  measure( );                                                     // start measurements
}

// Arduino main loop function, is executed cyclic
int8_t loopFn ( )
{
  int8_t res = APP_SUCCESS_OK;
  uint8_t intStatus = 0;
  int8_t exit = serialInput();   // handle any keystrokes from UART

#if ( defined( USE_INTERRUPT_TO_TRIGGER_READ ) && (USE_INTERRUPT_TO_TRIGGER_READ != 0) )
  if ( irqTriggered )
  {
    disableInterrupts( );
    irqTriggered = 0;
    enableInterrupts( );
 
#else
  if ( stateTmf8829 == TMF8829_STATE_MEASURE )
  { 
#endif

    intStatus = tmf8829GetAndClrInterrupts( &tmf8829, TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_HISTOGRAMS );
    if ( intStatus & TMF8829_APP_INT_RESULTS )    // check if a result is available
    {
      res = tmf8829ReadResults( &tmf8829 );
      updateOutput( );                            // after reading result configure update digital output based on activity region
    }
    if ( intStatus & TMF8829_APP_INT_HISTOGRAMS )
    {
      res = tmf8829ReadHistogram( &tmf8829);
    }
  }

  if ( res != APP_SUCCESS_OK ) // in case that fails there is some error in programming or on the device, this should not happen
  {
    tmf8829DisableInterrupts( &tmf8829, 0xFF );
    tmf8829StopMeasurement( &tmf8829 );
    stateTmf8829 = TMF8829_STATE_STOPPED;
    PRINT_CONST_STR( F(  "#Err" ) );
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR( F(  "inter" ) );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( intStatus );
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR( F(  "but no data" ) );
    PRINT_LN( );
  }
  server.handleClient();
  return !exit;    // 1 == loop again, 0 == exit
}

void terminateFn ( )
{
  tmf8829Disable( &tmf8829 );
  clrInterruptHandler( );

  i2cClose( &tmf8829 );
  inputClose( );
} 
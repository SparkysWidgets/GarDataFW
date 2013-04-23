#include <Wire.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DS3231RTC.h>
#include <DHT22.h>
#include <avr/eeprom.h>

const int lightRelay = 4;
bool lightState;
const int pumpRelay = 5;
bool pumpState;
const int obRelay1 = 8;
const int obPWM = 9;
const int obRelay2 = 2;

const int BATTSENSE = 16; //d16 = A2
const int DHTPIN = 6;

const int TIME_MSG_LEN = 11;
const int MAX_MSG_LEN = 16;
const char TIME_HEADER = 'T';
const char SCHEDULE_HEADER = 'S';
const char CONTROL_HEADER = 'C';
const char INFO_HEADER = 'I';

//EEPROM trigger check
#define Write_Check      0x1233
#define Version          0x0001

#define ADDRESS 0x4D // MCP3221(A5) I2C address
//Our parameters, for ease of use and eeprom access lets use a struct
struct parameters_T
{
  unsigned int WriteCheck;
  int lightOnHour,lightOnMin,lightOffHour,lightOffMin,
  pumpOnSec,pumpOffSec,sensorReadCycle,pH7Cal,pH4Cal;
  float pHStep;
} 
params;

//Globals
int adc_result;
float pH;
float vRef = 4.094; //Our vRef into the ADC wont be exactly 5v, best to measure and adjust here 
int systemTimer;
DHT22 myDHT22(DHTPIN);
AlarmId lightOnAlarmId, lightOffAlarmId, pumpAlarmId;

void setup()  
{
  eeprom_read_block(&params, (void *)0, sizeof(params));
  if (params.WriteCheck != Write_Check){
    reset_Params();
  }
  Wire.begin(); //conects I2C
  Serial.begin(57600);
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet)
  { 
    Serial.println("Unable to sync with the RTC");
  }
  else
  {
    Serial.println("RTC has set the system time");
  }
  
  lightOnAlarmId = Alarm.alarmRepeat(params.lightOnHour,params.lightOnMin,0,LightOn);
  lightOffAlarmId = Alarm.alarmRepeat(params.lightOffHour,params.lightOffMin,0,LightOff);
  pumpAlarmId = Alarm.timerOnce(params.pumpOffSec,PumpOn);
}

void loop()
{
  readpHADC();
  DHT22_ERROR_t errorCode;
  ReadSensors();
  //well avoid using delays by performing actions on time :)
  if(systemTimer > 1000)
  {
   systemTimer = 0;
   statusReport(); 
  }
  //If we have inbound serial lets process it
  if(Serial.available()) 
  {
    processMessage();
  }
  systemTimer++;
  Alarm.delay(0); // Important that this is called instead of delay and must be called for Alarms to be tracked
}

//All Messages will begin with their headers (then objects ie pump, light)
//and their respective actions or times seperate by commas
void processMessage()
{
	while(Serial.available())
	{
    char header = Serial.read();
	 switch(header)
	 {
		case TIME_HEADER:
		{
		 timeSyncMessage();
		}
		break;
	  
		case SCHEDULE_HEADER:
		{
		 scheduleMessage();
		}
		break;
	  
		case CONTROL_HEADER:
		{
		 controlMessage();
		}
		break;
	   
		case INFO_HEADER:
		{
		 parametersDump();
		}
		break;
	  
		default:
		{
	  
		}
		break; 
	   }
	}
}

void timeSyncMessage() {
  // if time sync available from serial port, update time and return true
  // time message consists of a header and ten ascii digits 
  while (Serial.available() > 0) {       
      time_t pctime = 0;
      for(int i=1; i < TIME_MSG_LEN -1; i++){   
        char c = Serial.read();          
        if( c >= '0' && c <= '9'){   
          pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
        }
      }   
      setTime(pctime);  // Sync Arduino clock to the time received on the serial port
      RTC.set(pctime); //sync RTC with our time, We may not always want to do this, BE SURE!
      } 
}

void scheduleMessage()
{
  char objToSchedule;
  int newtimes[4];
  //Since all of our schedule messages are they same format we can pull the time change info now
  //then simple adjust the proper schedule
  while (Serial.available() > 0) {
  //SL,08,00,18,00 -> this will change the lights schedule On Hour 08 On Minute 00, Off Hour 18 Off Minute 00
  objToSchedule = Serial.read();
  newtimes[0] = Serial.parseInt();  //If pump Hundreds/Thousands: On Seconds Decimal Places
  newtimes[1] = Serial.parseInt();  //If pump Ones/Tens:  On DP
  newtimes[2] = Serial.parseInt();  //If pump Hundreds/Thousands: OffDP
  newtimes[3] = Serial.parseInt();  //If pump Ones/Tens: Off DP
  }

  if(objToSchedule == 'L')
  {
  //Adjust our light schedule then
  //Since our schedule message looks like SL,AA,BB,CC,DD
  params.WriteCheck = Write_Check;
  params.lightOnHour = newtimes[0];
  params.lightOnMin = newtimes[1];
  params.lightOffHour = newtimes[2];
  params.lightOffMin = newtimes[3];
  eeprom_write_block(&params, (void *)0, sizeof(params)); 
  Alarm.write(lightOnAlarmId, AlarmHMS(params.lightOnHour, params.lightOnMin, 0));
  Alarm.write(lightOffAlarmId, AlarmHMS(params.lightOffHour, params.lightOffMin, 0));
  }

  if(objToSchedule == 'P')
  {
  //Adjust our pump schedule
  params.WriteCheck = Write_Check;
  params.pumpOnSec = (100*newtimes[0]) + newtimes[1];
  params.pumpOffSec = (100*newtimes[2]) + newtimes[3];
  eeprom_write_block(&params, (void *)0, sizeof(params));
  //Since the pump timer is "recycled each time all we have to do is update the times itll get on the next cycle 
  }
}

void controlMessage()
{
  char objToSchedule;
  while (Serial.available() > 0) {
  objToSchedule = Serial.read();
  }

  if(objToSchedule == 'L')
  {
    //If pin is low turn it on, otherwise turn it off
    if(lightState == false)
    {
     lightState = true;
     digitalWrite(lightRelay, HIGH); 
    }
    else
    {
     lightState = false;
     digitalWrite(lightRelay, LOW); 
    }
  }

  if(objToSchedule == 'P')
  {
    //If pin is low turn it on, otherwise turn it off
    if(pumpState == false)
    {
     pumpState = true;
     digitalWrite(pumpRelay, HIGH); 
    }
    else
    {
     pumpState = false;
     digitalWrite(pumpRelay, LOW); 
    }
  }
}

void parametersDump()
{
 // digital clock display of the time
  Serial.print(Version);
  Serial.print(',');
  Serial.print(params.lightOnHour);
  Serial.print(',');
  Serial.print(params.lightOnMin);
  Serial.print(',');
  Serial.print(params.lightOffHour);
  Serial.print(',');
  Serial.print(params.lightOffMin);
  Serial.print(',');
  Serial.print(params.pumpOnSec);
  Serial.print(',');
  Serial.print(params.pumpOffSec);
  Serial.print(',');
  Serial.print(params.sensorReadCycle);
  Serial.print(',');
  Serial.print(params.pHStep);
  Serial.print(',');
  Serial.print(params.pH7Cal);
  Serial.print(',');
  Serial.print(params.pH4Cal);
  Serial.println(); 
}

void ReadSensors()
{
  calcpH(adc_result);
  DHT22_ERROR_t errorCode;
  //Serial.print("Requesting data...");
  myDHT22.readData();
}

void readpHADC()
{
  //This is our I2C ADC interface section
  //We'll assign 2 BYTE variables to capture the LSB and MSB(or Hi Low in this case)
  byte adc_high;
  byte adc_low;
  Wire.requestFrom(ADDRESS, 2);        //requests 2 bytes
  while(Wire.available() < 2);         //while two bytes to receive
  //Set em 
  adc_high = Wire.read();           
  adc_low = Wire.read();
  //now assemble them, remembering our byte maths a Union works well here as well
  adc_result = (adc_high * 256) + adc_low;
}

//Now that we know our probe "age" we can calucalate the proper pH Its really a matter of applying the math
//We will find our milivolts based on ADV vref and reading, then we use the 7 calibration
//to find out how many steps that is away from 7, then apply our calibrated slope to calcualte real pH
void calcpH(int raw)
{
 float miliVolts = (((float)raw/4095)*vRef)*1000;
 float temp = ((((vRef*(float)params.pH7Cal)/4095)*1000)- miliVolts)/5.25; //5.25 is the gain of our amp stage we need to remove it
 pH = 7-(temp/params.pHStep);
}

//Lets read our raw reading while in pH7 calibration fluid and store it
//We will store in raw int formats as this math works the same on pH step calcs
void calibratepH7(int calnum)
{
  params.pH7Cal = calnum;
  calcpHSlope();
  //write these settings back to eeprom
  eeprom_write_block(&params, (void *)0, sizeof(params)); 
}

//Lets read our raw reading while in pH4 calibration fluid and store it
//We will store in raw int formats as this math works the same on pH step calcs
//Temperature compensation can be added by providing the temp offset per degree
//IIRC .009 per degree off 25c (temperature-25*.009 added pH@4calc)
void calibratepH4(int calnum)
{
  params.pH4Cal = calnum;
  calcpHSlope();
  //write these settings back to eeprom
  eeprom_write_block(&params, (void *)0, sizeof(params));
}

//This is really the heart of the calibration proccess, we want to capture the
//probes "age" and compare it to the Ideal Probe, the easiest way to capture two readings,
//at known point(4 and 7 for example) and calculate the slope.
//If your slope is drifting too much from Ideal(59.16) its time to clean or replace!
void calcpHSlope ()
{
  //RefVoltage * our deltaRawpH / 12bit steps *mV in V / OP-Amp gain /pH step difference 7-4
   params.pHStep = ((((vRef*(float)(params.pH7Cal - params.pH4Cal))/4096)*1000)/5.25)/3;
}


void statusReport(){
  // digital clock display of the time
  Serial.print(year());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(',');
  Serial.print(RTC.getTemp());
  Serial.print(',');
  Serial.print(myDHT22.getTemperatureC());
  Serial.print(',');
  Serial.print(myDHT22.getHumidity());
  Serial.print(',');
  Serial.print(pH);
  Serial.print(',');
  Serial.print(lightState);
  Serial.print(',');
  Serial.print(pumpState);
  Serial.print(',');
  Serial.print((float)analogRead(BATTSENSE)*vRef/1023*4.03);
  Serial.println();
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

//Our Timer Delegates
void LightOn()
{
 lightState = true;
 digitalWrite(lightRelay, HIGH); 
}

void LightOff()
{
 lightState = false;
 digitalWrite(lightRelay, LOW); 
}

void PumpOn()
{
 pumpState = true;
 digitalWrite(pumpRelay, HIGH);
 pumpAlarmId = Alarm.timerOnce(params.pumpOnSec,PumpOff);
}

void PumpOff()
{
 pumpState = false;
 digitalWrite(pumpRelay, LOW);
 pumpAlarmId = Alarm.timerOnce(params.pumpOffSec,PumpOn); 
}

//Reset paramter method
void reset_Params(void)
{

  params.WriteCheck = Write_Check;
  params.lightOnHour = 10;
  params.lightOnMin = 00;
  params.lightOffHour = 20;
  params.lightOffMin = 00;
  params.pumpOnSec = 5;
  params.pumpOffSec = 120;
  params.sensorReadCycle = 120;
  params.pHStep = 63.14;
  params.pH7Cal = 2260;
  params.pH4Cal = 1265;

  eeprom_write_block(&params, (void *)0, sizeof(params));
}

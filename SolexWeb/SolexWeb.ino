#include <Arduino.h>
#include "WiFi.h"
#include <ESPAsyncWebServer.h> # Direct download from https://github.com/me-no-dev/ESPAsyncWebServer, LibMgr install has bugs
#include <ArduinoJson.h>
#include <Wire.h>

// IMPORTANT !!
#define initmemo 1  // Set to 1 during the first programming for memory initialization. Then set initmemo to 0

#define MOTOR2_PIN_1  32   // Blue   - 28BYJ48 pin 1
#define MOTOR2_PIN_3  33  // Pink   - 28BYJ48 pin 2
#define MOTOR2_PIN_2   25  // Yellow - 28BYJ48 pin 3
#define MOTOR2_PIN_4  26   //  Orange - 28BYJ48 pin 4

#define MOTOR3_PIN_1  27   // Blue   - 28BYJ48 pin 1
#define MOTOR3_PIN_3  14  // Pink   - 28BYJ48 pin 2
#define MOTOR3_PIN_2   12  // Yellow - 28BYJ48 pin 3
#define MOTOR3_PIN_4  13   //  Orange - 28BYJ48 pin 4

#define MOTOR_PIN_1  19   // Blue   - 28BYJ48 pin 1
#define MOTOR_PIN_3  18  // Pink   - 28BYJ48 pin 2
#define MOTOR_PIN_2   5  // Yellow - 28BYJ48 pin 3
#define MOTOR_PIN_4  17   //  Orange - 28BYJ48 pin 4

#define adresse_EEPROM 0x50   // external eeprom 24LC01B
#define Adr1_RW_EEPROM 0
#define Adr2_RW_EEPROM 1

// Presets
unsigned int RAIEHa = 1230;
unsigned int RAIENa = 1060;
unsigned int RAIEMg = 910;
unsigned int RAIEHb = 850;
unsigned int RAIECa = 670;

const char *SSID = "SunriseGuest";
const char *PWD = "Summer0824Sunrise";

AsyncWebServer server(80);

String header;
const unsigned int STEPZero = 0;  

const unsigned int STEPRes1 = 5;  
const unsigned int STEPRes2 = 25;  
const unsigned int STEPRes3 = 100;  

const unsigned int STEPCam1 = 5;  
const unsigned int STEPCam2 = 25; 

const unsigned int STEPFoc1 = 5;  
const unsigned int STEPFoc2 = 50; 
const unsigned int STEPFoc3 = 200; 

const unsigned int Backlash_Res = 15;  
const unsigned int Backlash_Cam = 15 + 25;  // pinion sets (15) and helical focus sets. (25) To be adjusted if necessary      
const unsigned int Backlash_Foc = 15;  


int unsigned STEPREL = 0;
unsigned int indice_ray_select = 0;

const unsigned long STEP_DELAY_MICROSEC = 5000;  // motor speed
int cycle = 0;
int cycle2 = 0;
int cycle3 = 0;

bool direction = true ;
bool dir_Cam = true ;
bool dir_Foc = true ;
bool dir_Res = true ;

int positionRes = 0 ;
int positionCam = 0 ;
int positionFoc = 0 ;
unsigned long last_step_time;

bool memo = false;
int address = 0x50;
bool CouplageOk = false;

char buffer[250];

// Static properties
static String DEVICE_NAME = "Sol'Ex Trimotor";
static String SOFTWARE_VERSION = "0.0.1";
static String PROTOCOL_VERSION = "0.0.1";
static String GRATING_ID="0";
static String SLIT_ID="0";
static String SLIT_WIDTH="10";

// Define the function to handle GET requests
void handleGetRequest(AsyncWebServerRequest *request) {
  StaticJsonDocument<200> doc;
  String propertyValue;
  String message;

  // Provide the requested data
  if (request->hasParam("property")) {
    // Static values
    if      (request->getParam("property")->value()=="DEVICE_NAME")       { doc["property"]="DEVICE_NAME";      doc["value"]=DEVICE_NAME; } 
    else if (request->getParam("property")->value()=="SOFTWARE_VERSION")  { doc["property"]="SOFTWARE_VERSION"; doc["value"]=SOFTWARE_VERSION; }
    else if (request->getParam("property")->value()=="PROTOCOL_VERSION")  { doc["property"]="PROTOCOL_VERSION"; doc["value"]=PROTOCOL_VERSION; }
    else if (request->getParam("property")->value()=="TEMPERATURE")       { doc["property"]="TEMPERATURE";      doc["value"]="Not Supported"; }
    else if (request->getParam("property")->value()=="HUMIDITY")          { doc["property"]="HUMIDITY";         doc["value"]="Not Supported"; }
    else if (request->getParam("property")->value()=="GRATING_ID")        { doc["property"]="GRATING_ID";       doc["value"]=GRATING_ID; }
    else if (request->getParam("property")->value()=="SLIT_ID")           { doc["property"]="SLIT_ID";          doc["value"]=SLIT_ID; } 
    else if (request->getParam("property")->value()=="SLIT_WIDTH")        { doc["property"]="SLIT_WIDTH";       doc["value"]=SLIT_WIDTH; }

    // Dynamic Values
    else if (request->getParam("property")->value()=="GRATING_ANGLE")     { doc["property"]="GRATING_ANGLE";    doc["value"]="999"; }
    else if (request->getParam("property")->value()=="GRATING_WAVELENGTH"){ doc["property"]="GRATING_WAVELENGTH"; doc["value"]="999"; }
    else if (request->getParam("property")->value()=="SLIT_ANGLE")        { doc["property"]="SLIT_ANGLE";       doc["value"]= "999"; }
    else if (request->getParam("property")->value()=="FOCUS_POSITION")    { doc["property"]="FOCUS_POSITION";   doc["value"]= "999"; }
    else if (request->getParam("property")->value()=="LIGHT_SOURCE")      { doc["property"]="LIGHT_SOURCE";     doc["value"]= "999"; }
    serializeJson(doc, message);
    request->send(200, "application/json", message);
    }
  else 
    {
    request->send(400, "application/json", "{\"error\":\"No properties requested\"}");
    }
}

void handleSetRequest(AsyncWebServerRequest *request) {
  StaticJsonDocument<200> doc;
  String propertyValue;
  String message;

  // Provide the requested data
  if (request->hasParam("property")) {
    if (request->getParam("property")->value()=="GRATING_ANGLE")     
    { 
      // Set the grating angle

      // Return what we set it to
      doc["property"]="GRATING_ANGLE";
      doc["value"]="999"; }
    else if (request->getParam("property")->value()=="GRATING_WAVELENGTH")
    { 
      // Set the grating wavelength

      // Return what we set it to
      doc["code"]="M00";
      doc["property"]="GRATING_WAVELENGTH"; 
      doc["value"]="999"; 
    }
    else if (request->getParam("property")->value()=="FOCUS_POSITION_TELESCOPE")    
      { 
        // Set the focus position for the Telescope

        // Return what we set it to
        doc["code"]="M00";
        doc["property"]="FOCUS_POSITION";   
        doc["value"]= "999"; 
      }
    else if (request->getParam("property")->value()=="FOCUS_POSITION_SHG")    
      { 
        // Set the focus position for the Telescope
        
        // Return what we set it to
        doc["code"]="M00";
        doc["property"]="FOCUS_POSITION";   
        doc["value"]= "999"; 
      }
    serializeJson(doc, message);
    request->send(200, "application/json", message);
    }
  else 
    {
    doc["code"]="M01";
    doc["property"]=request->getParam("property")->value();   
    serializeJson(doc, message);
    request->send(400, "application/json", message);
    }
}
void setup_routing() {
  // Standard command format  COMMAND;PARAMETER;ATTRIBUTE;VALUE, so COMMAND is via URL
  server.on("/get", HTTP_GET, handleGetRequest);  // GET endpoint
  server.on("/set", HTTP_GET, handleSetRequest);  // SET endpoint

  server.begin();    
}

void setup() {     
  Serial.begin(115200); 
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(SSID, PWD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
  setup_routing();     

  // Initialize serial port I/O.
  Serial.println("Sol'Ex 3 motor driver begins\n");
  Wire.begin();

  // Initialize motor control pins...
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(MOTOR_PIN_3, OUTPUT);
  pinMode(MOTOR_PIN_4, OUTPUT);
  pinMode(MOTOR2_PIN_1, OUTPUT);
  pinMode(MOTOR2_PIN_2, OUTPUT);
  pinMode(MOTOR2_PIN_3, OUTPUT);
  pinMode(MOTOR2_PIN_4, OUTPUT);
  pinMode(MOTOR3_PIN_1, OUTPUT);
  pinMode(MOTOR3_PIN_2, OUTPUT);
  pinMode(MOTOR3_PIN_3, OUTPUT);
  pinMode(MOTOR3_PIN_4, OUTPUT);

  direction = true;
  dir_Cam = true;
  dir_Res = true;
  dir_Foc = true;
   
  last_step_time = 0L;
  
  #if initmemo
     for(int i = 0 ; i < 30; i++){   // set the memory to 0 for the first use. Then reset initmemo to 0
       writebyte(adresse_EEPROM,i,0);
     }
  indice_ray_select = 20;  positionRes=RAIEHa; WriteMemoryRes(indice_ray_select);
  indice_ray_select = 22;  positionRes=RAIENa; WriteMemoryRes(indice_ray_select);
  indice_ray_select = 24;  positionRes=RAIEMg; WriteMemoryRes(indice_ray_select);
  indice_ray_select = 26;  positionRes=RAIEHb; WriteMemoryRes(indice_ray_select);
  indice_ray_select = 28;  positionRes=RAIECa; WriteMemoryRes(indice_ray_select);
  #endif
  byte val1 = readbyte(adresse_EEPROM, 0);
  byte val2 = readbyte(adresse_EEPROM, 1);
  positionRes = word(val1, val2);
  cycle = readbyte(adresse_EEPROM, 2);
  dir_Res = readbyte(adresse_EEPROM, 3);

  val1 = readbyte(adresse_EEPROM, 4);
  val2 = readbyte(adresse_EEPROM, 5);
  positionCam = word(val1, val2);
  cycle2 = readbyte(adresse_EEPROM, 6);
  dir_Cam = readbyte(adresse_EEPROM,7); 
     
  val1 = readbyte(adresse_EEPROM, 8);
  val2 = readbyte(adresse_EEPROM, 9);
  positionFoc = word(val1, val2);
  cycle3 = readbyte(adresse_EEPROM, 10);
  dir_Foc = readbyte(adresse_EEPROM, 11);
}    
       
void loop() {   
}

/*-----------------------------------------
 Routines for the 3 28BYJ48 motors 
 -------------------------------------------*/
void commandMotor(unsigned int st,unsigned int mt)  // St number of steps - mt motor number 0: Grating 1: Camera 2: Telescope
{  
  switch (mt){
      case 1:
        if (direction != dir_Res )  { st = st + Backlash_Res; }  
        dir_Res = direction ;         
        break;
      case 2:
        if (direction != dir_Cam )  { st = st + Backlash_Cam; }  
        dir_Cam = direction ;           
        break;
      case 3:
        if  (direction != dir_Foc )  { st = st + Backlash_Foc; }  
        dir_Foc = direction ;         
        break;
  }         
  for(int i=0;i<st;i++){
		Motor(mt);
		// tempo to set the motor speed
		last_step_time = micros();
		while(micros() - last_step_time < STEP_DELAY_MICROSEC){}
	}
	stop(mt);  
	setMemory(mt); // memorize position
}

/* ----------------------------------------------------------------------
Motor sending 1 to 3 (mt) on a position (st) in number of steps
---------------------------------------------------------------------------*/
void Goto(unsigned int st,unsigned int mt) 
{
   switch (mt){
      case 1:
      if(positionRes != st)  // motor movement if the motor is not already in the same position.
      {
        if (positionRes < st)
        {
          direction = true;
          STEPREL = st-positionRes;
          positionRes = st;
          commandMotor(STEPREL,mt);                                
        }
      else
        {
          direction = false;
          STEPREL = positionRes-st;
          positionRes = st;
          commandMotor(STEPREL,mt);  
        }
      }
      break;
    case 2:
     if(positionCam != st)  // motor movement if the motor is not already in the same position.
      {
      if (positionCam < st)
      {
        direction = true;
        STEPREL = st-positionCam;
        positionCam = st;
        commandMotor(STEPREL,mt);                                
      }
      else
      {
        direction = false;
        STEPREL = positionCam-st;
        positionCam = st;
        commandMotor(STEPREL,mt);  
      }
     }
     break;
   }
}


void Motor(unsigned int mt)
{
if (direction == true){
    switch (mt){
      case 1:
        stepMotorRes();
        cycle++;
		    if (cycle == 4){ 
		    	cycle = 0;
		    } 
        break;
      case 2:
        stepMotorCam(); 
        cycle2++;
		    if (cycle2 == 4){ 
		    	cycle2 = 0;
		    }         
        break;
      case 3:
        stepMotorFoc(); 
        cycle3++;
		    if (cycle3 == 4){ 
		    	cycle3 = 0;
		    }      
        break;
    }
	} else{
    switch (mt){
      case 1:
        stepMotorRes();
        if (cycle == 0){ 
		    	cycle = 4 ;
	    	}
	 	    cycle--;
        break;
      case 2:
        stepMotorCam();
        if (cycle2 == 0){ 
		    	cycle2 = 4 ;
	    	}
	 	    cycle2--;         
        break;
      case 3:
        stepMotorFoc(); 
      if (cycle3 == 0){ 
		    	cycle3 = 4 ;
	    	}
	 	    cycle3--;         
        break;
    }
	}
}



void stepMotorRes() // Network
{
	switch (cycle){
		case 0: // 1010
			digitalWrite(MOTOR_PIN_1, HIGH);
			digitalWrite(MOTOR_PIN_2, LOW);
			digitalWrite(MOTOR_PIN_3, HIGH);
			digitalWrite(MOTOR_PIN_4, LOW);
			break;
		case 1: // 0110
			digitalWrite(MOTOR_PIN_1, LOW);
			digitalWrite(MOTOR_PIN_2, HIGH);
			digitalWrite(MOTOR_PIN_3, HIGH);
			digitalWrite(MOTOR_PIN_4, LOW);
			break;
		case 2: // 0101
			digitalWrite(MOTOR_PIN_1, LOW);
			digitalWrite(MOTOR_PIN_2, HIGH);
			digitalWrite(MOTOR_PIN_3, LOW);
			digitalWrite(MOTOR_PIN_4, HIGH);
			break;
		case 3: // 1001
			digitalWrite(MOTOR_PIN_1, HIGH);
			digitalWrite(MOTOR_PIN_2, LOW);
			digitalWrite(MOTOR_PIN_3, LOW);
			digitalWrite(MOTOR_PIN_4, HIGH);
			break;
	}
}

void stepMotorCam() // Camera
{
  switch (cycle2){
    case 0: // 1010
      digitalWrite(MOTOR3_PIN_1, HIGH);
      digitalWrite(MOTOR3_PIN_2, LOW);
      digitalWrite(MOTOR3_PIN_3, HIGH);
      digitalWrite(MOTOR3_PIN_4, LOW);
      break;
    case 1: // 0110
      digitalWrite(MOTOR3_PIN_1, LOW);
      digitalWrite(MOTOR3_PIN_2, HIGH);
      digitalWrite(MOTOR3_PIN_3, HIGH);
      digitalWrite(MOTOR3_PIN_4, LOW);
      break;
    case 2: // 0101
      digitalWrite(MOTOR3_PIN_1, LOW);
      digitalWrite(MOTOR3_PIN_2, HIGH);
      digitalWrite(MOTOR3_PIN_3, LOW);
      digitalWrite(MOTOR3_PIN_4, HIGH);
      break;
    case 3: // 1001
      digitalWrite(MOTOR3_PIN_1, HIGH);
      digitalWrite(MOTOR3_PIN_2, LOW);
      digitalWrite(MOTOR3_PIN_3, LOW);
      digitalWrite(MOTOR3_PIN_4, HIGH);
      break;
  }
}


void stepMotorFoc() // Telescope
{
  switch (cycle3){
    case 0: // 1010
      digitalWrite(MOTOR2_PIN_1, HIGH);
      digitalWrite(MOTOR2_PIN_2, LOW);
      digitalWrite(MOTOR2_PIN_3, HIGH);
      digitalWrite(MOTOR2_PIN_4, LOW);
      break;
    case 1: // 0110
      digitalWrite(MOTOR2_PIN_1, LOW);
      digitalWrite(MOTOR2_PIN_2, HIGH);
      digitalWrite(MOTOR2_PIN_3, HIGH);
      digitalWrite(MOTOR2_PIN_4, LOW);
      break;
    case 2: // 0101
      digitalWrite(MOTOR2_PIN_1, LOW);
      digitalWrite(MOTOR2_PIN_2, HIGH);
      digitalWrite(MOTOR2_PIN_3, LOW);
      digitalWrite(MOTOR2_PIN_4, HIGH);
      break;
    case 3: // 1001
      digitalWrite(MOTOR2_PIN_1, HIGH);
      digitalWrite(MOTOR2_PIN_2, LOW);
      digitalWrite(MOTOR2_PIN_3, LOW);
      digitalWrite(MOTOR2_PIN_4, HIGH);
      break;
  }
}

void stop(unsigned int mt) 
{
// stops the motor, without power. (limits consumption)
switch (mt){
    case 1:
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, LOW);
      digitalWrite(MOTOR_PIN_3, LOW);
      digitalWrite(MOTOR_PIN_4, LOW);
      break;
    case 2:
      digitalWrite(MOTOR2_PIN_1, LOW);
      digitalWrite(MOTOR2_PIN_2, LOW);
      digitalWrite(MOTOR2_PIN_3, LOW);
      digitalWrite(MOTOR2_PIN_4, LOW);
      break;
    case 3:
      digitalWrite(MOTOR3_PIN_1, LOW);
      digitalWrite(MOTOR3_PIN_2, LOW);
      digitalWrite(MOTOR3_PIN_3, LOW);
      digitalWrite(MOTOR3_PIN_4, LOW);
      break;
  }
}

/* ---------------------------------------------------------------------------
Read/write grating position and camera focus
-------------------------------------------------------------------------------*/
void WriteMemoryCam(unsigned int IndiceRaie)
{
  byte hi;
  byte low;
  hi  = highByte(positionCam);  // save last position in eeprom
  low = lowByte(positionCam);
  writebyte(adresse_EEPROM, IndiceRaie, hi);
  writebyte(adresse_EEPROM, IndiceRaie + 1, low);
}

unsigned int ReadMemoryCam(unsigned int IndiceRaie)
{
  byte dat1 = readbyte(adresse_EEPROM, IndiceRaie);
  byte dat2 = readbyte(adresse_EEPROM, IndiceRaie + 1);
  return  word(dat1, dat2);
}

void WriteMemoryRes(unsigned int IndiceRaie)
{
  byte hi;
  byte low;
  hi  = highByte(positionRes);  // save last position in eeprom
  low = lowByte(positionRes);
  writebyte(adresse_EEPROM, IndiceRaie+10, hi);
  writebyte(adresse_EEPROM, IndiceRaie + 11, low);
}

unsigned int ReadMemoryRes(unsigned int IndiceRaie)
{
  byte dat1 = readbyte(adresse_EEPROM, IndiceRaie+10);
  byte dat2 = readbyte(adresse_EEPROM, IndiceRaie + 11);
  return  word(dat1, dat2);
}

/* -------------------------------------------
Storing the position of a motor
-----------------------------------------------*/
void setMemory(unsigned int mt)
{
  // Memory of the last position of the motor used.
   byte hi;
   byte low;
  switch (mt){
    case 1:
        hi = highByte(positionRes);  // // save last position in eeprom
        low = lowByte(positionRes);
        writebyte(adresse_EEPROM, 0, hi);
        writebyte(adresse_EEPROM, 1, low);
        writebyte(adresse_EEPROM, 2, cycle);
        writebyte(adresse_EEPROM, 3, dir_Res );
        break;
    case 2:
        hi = highByte(positionCam);  // // save last position in eeprom
        low = lowByte(positionCam);
        writebyte(adresse_EEPROM, 4, hi);
        writebyte(adresse_EEPROM, 5, low);
        writebyte(adresse_EEPROM, 6, cycle2);
        writebyte(adresse_EEPROM, 7, dir_Cam );
         break;
    case 3: 
        hi = highByte(positionFoc);  // // save last position in eeprom
        low = lowByte(positionFoc);
        writebyte(adresse_EEPROM, 8, hi);
        writebyte(adresse_EEPROM, 9, low);
        writebyte(adresse_EEPROM, 10, cycle3);
        writebyte(adresse_EEPROM, 11, dir_Foc );  
        break;
	}
}

/* -------------------------------------------
Reading the 24LC01B EEPROM
-----------------------------------------------*/
byte readbyte(int adressei2c, unsigned int adresseMem )
{
	byte lecture = 0;

	Wire.beginTransmission(adressei2c);  // i2c address of 24LC01B
	Wire.write(adresseMem); 		// address of the byte you want to read
	Wire.endTransmission();
	Wire.requestFrom(adressei2c, 1); // request to read the byte
	delay(5);
	if (Wire.available()) {  
		lecture = Wire.read(); // reading the info
	}
  return lecture;
}

/* -------------------------------------------
Writing the 24LC01B EEPROM
-----------------------------------------------*/
void writebyte(int adressei2c, unsigned int adresseMem, byte data )
{
	 Wire.beginTransmission(adressei2c); // i2c address of 24LC01B
	 Wire.write(adresseMem); // address of the byte you want to write
	 Wire.write(data);		 // write it
	 Wire.endTransmission(); // finish
	 delay(5);
}

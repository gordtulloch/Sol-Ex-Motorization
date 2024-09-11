// program for Sol'Ex (solar explorer) by Christian Buil.
// original version: Jean Brunet on 01/17/2023 written to order the movement of the network.
// This version: Gord Tulloch 2024/7/30 translated to English

// This new version has been rewritten since 04/22/2023 by Pascal Berteau with 5 programmed lines 
// and the control of 3 motors, network, camera focus and telescope focus.
// The program was the subject of numerous additions by Jean Brunet and Pascal Berteau to finally
// lead to this one.

// The position of the lines is memorized, as well as the position of the camera focus by pressing Memo.
// The coupling of the network and camera focus motors allows you to position yourself on a line by automatically obtaining sharpness.
// So there is no need to intervene manually on Sol'Ex, nor on the riflescope which uses the 3rd motor.

// This program is to be chosen for the 24LC01B EEPROM memory.
// If you use an AT24Cxx module, take the program ESP32_SolEx_AT24Cxx.ino

// IMPORTANT !!
#define initmemo 1  // Set to 1 during the first programming for memory initialization. Then set initmemo to 0

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <WiFi.h>

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

// raies programmées

unsigned int RAIEHa = 1230;
unsigned int RAIENa = 1060;
unsigned int RAIEMg = 910;
unsigned int RAIEHb = 850;
unsigned int RAIECa = 670;


const char* ssid     = "SolEx";
const char* password = "solex1234";  // TO MODIFY if several Sol'Ex nearby
WiFiServer server(80);

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
/* ---------------------------
setup
-------------------------------*/
void setup() 
{
    // Initialize serial port I/O.
    //Serial.begin(115200); // only for debugging
    //Serial.println("Sol'Ex 3 motor driver begins\n");

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

   // WiFi.mode(WIFI_AP);
   // WiFi.setTxPower(WIFI_POWER_5dBm);
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    //Serial.print("IP Address is ");
    //Serial.println(IP);

    server.begin();
    // // reading from EEPROM memory of the last recorded position.
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
 /* --------------------------------
Processing loop
-------------------------------------*/
void loop() 
{
	Wificommmand();  // display of the web page and processing of user feedback
}


/*---------------------------------------------------------------------------------------------
 Web page and processing of orders sent by the user
------------------------------------------------------------------------------------------------*/
void Wificommmand()
{

	WiFiClient client = server.available();   // Listen for incoming clients
  CouplageOk = false; 
	if (client){ 
   while (client.connected()){            // loop while the client's connected
		if (client.available()) {             // if there's bytes to read from the client,
			char c = client.read();             // read a byte, then
			header += c;
			if (c == '\n') // if it is the end of the order received
			{          
				client.println("HTTP/1.1 200 OK");
				client.println("Content-type:text/html");
				client.println("Connection: close");
				client.println();

			  if (header.indexOf("couplage") >= 0){  // coupling is checked, the camera motor is functional
					  CouplageOk = true;
				}
        // manual displacement corrections 
				if (header.indexOf("GPR") >= 0){ // Left / Small Step / Network
            direction = false;
            positionRes = positionRes-STEPRes1;
            commandMotor(STEPRes1,1);
				}
				else if (header.indexOf("GMR") >= 0){  // Left Middle Network
            direction = false;
            positionRes = positionRes-STEPRes2;
            commandMotor(STEPRes2,1);
         }
         else if (header.indexOf("GGR") >= 0){ // Left Large Network
            direction = false;
            positionRes = positionRes-STEPRes3;
            commandMotor(STEPRes3,1);
         }
				else if (header.indexOf("DPR") >= 0){    // Right / Small Step / Network
            direction = true;
            positionRes = positionRes+STEPRes1;  
            commandMotor(STEPRes1,1);
				 }
				 else if (header.indexOf("DMR") >= 0){ // Right / Medium Steps / Network
            direction = true;
            positionRes = positionRes+STEPRes2;  
            commandMotor(STEPRes2,1);	 
				 }
          else if (header.indexOf("DGR") >= 0){ // Right / Big Step / Network
            direction = true;
            positionRes = positionRes+STEPRes3;  
            commandMotor(STEPRes3,1);	 
				// ---------------------------
        	 } 
          else if (header.indexOf("MemoZ") >= 0){ // Memorization order 0
            positionRes = 0;
         }
         else if (header.indexOf("OrdreZ") >= 0){  // Return to order 0          
            Goto(STEPZero,1);
				  }
          else if (header.indexOf("CamInF") >= 0){ // Focus Camera in <<
					  direction = false;
            positionCam = positionCam-STEPCam2;  
				    commandMotor(STEPCam2,2);	 
				 }
        else if (header.indexOf("CamInS") >= 0){ //Focus Camera in <
					  direction = false;
            positionCam = positionCam-STEPCam1;  
				    commandMotor(STEPCam1,2);	 
				 }
          else if (header.indexOf("CamOutF") >= 0){ //Focus Camera Out >>
            direction = true;
            positionCam = positionCam+STEPCam2;  
            commandMotor(STEPCam2,2);	 
				 }
          else if (header.indexOf("CamOutS") >= 0){ //Focus Camera Out >
            direction = true;
            positionCam = positionCam+STEPCam1;  
            commandMotor(STEPCam1,2);	 
				 } 
          else if (header.indexOf("MemoCam") >= 0){   // memorize the focus for the current line.
          // record the position of the Cam motor for a selected line    
            WriteMemoryCam(indice_ray_select);
            WriteMemoryRes(indice_ray_select);
          }
          else if (header.indexOf("FocInX") >= 0) { //Focus Telescope in <<<
					   direction = false;
             positionFoc=positionFoc-STEPFoc3;  
				    commandMotor(STEPFoc3,3);	 
				 } 
          else if (header.indexOf("FocInF") >= 0){ //Focus Telescope in <<
					  direction = false;
            positionFoc = positionFoc-STEPFoc2;  
				    commandMotor(STEPFoc2,3);	 
				 }
        else if (header.indexOf("FocInS") >= 0){ //Focus Telescope in <
            direction = false;
            positionFoc = positionFoc-STEPFoc1;  
            commandMotor(STEPFoc1,3);	 
				 }
         else if (header.indexOf("FocOutX") >= 0) { //Focus Telescope Out >>>
            direction = true;
            positionFoc=positionFoc+STEPFoc3;  
            commandMotor(STEPFoc3,3);	 
				 }
          else if (header.indexOf("FocOutF") >= 0){ //Focus Telescope Out >>
            direction = true;
            positionFoc = positionFoc+STEPFoc2;  
            commandMotor(STEPFoc2,3);	 
				 }
          else if (header.indexOf("FocOutS") >= 0){ //Focus Telescope Out >
					  direction = true;
            positionFoc = positionFoc+STEPFoc1;  
				    commandMotor(STEPFoc1,3);	 
				 }  
          else if (header.indexOf("Ca") >= 0){ // Network position on defined lines 
            indice_ray_select = 28; 
            RAIECa = ReadMemoryRes(indice_ray_select);
            Goto(RAIECa,1); 
            if(CouplageOk == true){
              Goto(ReadMemoryCam(indice_ray_select),2);
            }
          }
          else if (header.indexOf("Hb") >= 0){
           indice_ray_select = 26; 
           RAIEHb = ReadMemoryRes(indice_ray_select);
           Goto(RAIEHb,1);
           if(CouplageOk == true){
            Goto(ReadMemoryCam(indice_ray_select),2); 
            }     
          }
          else if (header.indexOf("Mg") >= 0){
           indice_ray_select = 24; 
           RAIEMg = ReadMemoryRes(indice_ray_select);
           Goto(RAIEMg,1); 
           if(CouplageOk == true){
            Goto(ReadMemoryCam(indice_ray_select),2);	 
            }
          }
          else if (header.indexOf("Na") >= 0){ 
           indice_ray_select = 22; 
           RAIENa = ReadMemoryRes(indice_ray_select);
           Goto(RAIENa,1);
           if(CouplageOk == true){
             Goto(ReadMemoryCam(indice_ray_select),2);	
             }
          }
          else if (header.indexOf("Ha") >= 0){ 
            indice_ray_select = 20; 
            RAIEHa = ReadMemoryRes(indice_ray_select);
            Goto(RAIEHa,1); 
            if(CouplageOk == true){
              Goto(ReadMemoryCam(indice_ray_select),2);	
             }
          }
       
       	client.println(webPage());  // page web
				// The HTTP response ends with another blank line
      	client.println();
				client.stop();
				header = "";
        } 
		  }
		}
	}
}

/* --------------------
Affichage page web
----------------------*/

String webPage()
{
  String Ch;
  Ch = "<!DOCTYPE html><html>";
  Ch +="<head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  Ch +="<style>.bouton {background-color: #504caf; border: none; color: white; width:60px; height:50px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 15px; cursor: pointer; }"; 
  Ch +=".bouton2 {background-color: #504caf; border: none; color: white; width:105px; height:50px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 15px; cursor: pointer; }"; 
  Ch +=".boutonLite {background-color: #504caf; border: none; color: white; width:65px; height:65px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 22px; cursor: pointer; }"; 
  Ch +=".boutonLite2 {background-color: #504caf; border: none; color: white; width:50px; height:70px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 22px; cursor: pointer; }"; 
  Ch +=".texteEntree {background-color: #EEEEEE; border: none; color: black; width:165px; height:30px;  padding: 10px; text-decoration: none; text-align: left; display: inline-block; font-size: 22px; cursor: pointer; }</style>"; 
  Ch +="</head><body>";    
  
  Ch +="<div style='text-align: center '><H2>Sol'Ex</H2><form enctype='multipart/form data' method=GET>"; 
  Ch +="<H4>--- Grating " + String(positionRes)+ " --- </H4></center>";
  Ch +="<input button class='boutonLite2' type='submit' name='GGR' value = '<<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='GMR' value = '<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='GPR' value = '<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='DPR' value = '>' />";   
  Ch +="<input button class='boutonLite2' type='submit' name='DMR' value = '>>' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='DGR' value = '>>>' /><br><br>"; 
  Ch +="<input button class='bouton' type='submit' name='Ca' value = 'Ca' />";  
  Ch +="<input button class='bouton' type='submit' name='Hb' value =  'H &#946 ' />";  
  Ch +="<input button class='bouton' type='submit' name='Mg' value = 'Mg' />";  
  Ch +="<input button class='bouton' type='submit' name='Na' value = 'Na' />"; 
  Ch +="<input button class='bouton' type='submit' name='Ha' value = 'H &#945 ' /><br><br>";

  String op = ""; 
  if (CouplageOk == true){
     op = "checked= 'yes'" ;
  }  
  
  Ch +="<input button class='bouton2' type='submit' name='MemoZ' value = 'Set 0' />";
  Ch +="<input button class='bouton2' type='submit' name='OrdreZ' value = 'Order 0' />";  
  Ch +="<input button class='bouton2' type='submit' name='MemoCam' value = 'Save' /><br>"; 

  Ch +="<H4>--- Camera " + String(positionCam)+ " --- </H4></center>";
  Ch +="<input button class='boutonLite' type='submit' name='CamInF' value = '<<' />"; 
  Ch +="<input button class='boutonLite' type='submit' name='CamInS' value = '<' />"; 
  Ch +="<input button class='boutonLite' type='submit' name='CamOutS' value = '>' />";   
  Ch +="<input button class='boutonLite' type='submit' name='CamOutF' value = '>>' /><br><br>";
  Ch +="Connected <input type='checkbox' name='couplage' " + op + " value='couplage'/><br>";
 
  Ch +="<H4>--- Focuser " + String(positionFoc)+ " --- </H4></center>";
  Ch +="<input button class='boutonLite2' type='submit' name='FocInX' value = '<<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocInF' value = '<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocInS' value = '<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocOutS' value = '>' />";   
  Ch +="<input button class='boutonLite2' type='submit' name='FocOutF' value = '>>' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocOutX' value = '>>>' /><br>"; 
  Ch +="</form> </div></body></html>"; 
  return Ch;
}


/*-----------------------------------------
 Routines des 3 moteurs 28BYJ48
 -------------------------------------------*/
void commandMotor(unsigned int st,unsigned int mt)  // St number of steps - mt motor number 0: Network 1: Cam 2: Telescope
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
Read/write network position and camera focus
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

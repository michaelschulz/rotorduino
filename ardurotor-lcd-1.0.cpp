/* 
Simple 3-wire rotor controller for tracking satellites. 
This code uses an existing AZ only simple rotator to track satellites using the EASYCOM I 
protocol. 

This code was written by Michael Schulz (K5TRI) and is licensed under the GPLv2
Comments and improvements welcome at mschulz@creative-chaos.com



GPredict users: 

In the antenna controller you need to set the Tolerance to 0.01. If left at 
5.0 degrees, the value sent to the controller on switchover from 0 deg to 360 will go back
and forth causing the rotor to do two full turns!


*/


#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <Wire.h>
//#include <Adafruit_MCP23017.h>
//#include <Adafruit_RGBLCDShield.h>

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.

//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();



// These #defines make it easy to set the backlight color for use with Adafruit I2c LCD shield

/*#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
*/

struct azimuth_t
{
    int az_cur;
} azim;



// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11,10, 6, 5);

	unsigned char relayEXEpin = 9;
	unsigned char relayROTpin = 8;
	unsigned char rotButCCW = 3;
	unsigned char rotButCW = 4;
	char inputData[19];
	int s;
	int lastexe;
	int idle;
	int startidle;
    int az_temp;
    int b;
	int predictInput[12]; 
	byte startbyte;
	int ButtonState = 0;
	unsigned long time_on;
	unsigned long time_off;
	unsigned long time_diff;

	int ButtonCW();
	int ButtonCCW();
	
	
void setup()

{

  // retrieve stored position from memory
  // int az_cur;

  EEPROM_readAnything(0, azim.az_cur);

  // Make sure rotor is turned off
  digitalWrite(relayEXEpin,LOW);
 
 // set up the LCD's number of columns and rows: 

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.clear();

  //display current postion of rotor

  
  //Idle message while waiting for commands
  lcd.clear();
  lcd.print("K5TRI RotorDuino");
  lcd.setCursor(0,1);
  lcd.print("Current pos: ");
  lcd.print(azim.az_cur);

// initialize the serial communications:

  Serial.begin(9600);

// define relay pins for rotor control

    pinMode(ledCCWpin,OUTPUT);
    pinMode(ledCWpin,OUTPUT);
    pinMode(relayEXEpin,OUTPUT);
	pinMode(relayROTpin,OUTPUT);
   
	pinMode(rotButCCW,INPUT);
	pinMode(rotButCW,INPUT);
    

}

void loop()

	{
 

		startidle = (millis() /1000);

		
		EEPROM_readAnything(0, azim.az_cur); // Get current position stored in EEPROM
		azim.az_cur = constrain(azim.az_cur, 0, 360); //Limit rotor positioning between 0 and 360 degrees
 
		//manual CCW button pressed
		if (digitalRead(rotButCCW) == HIGH) 
		{
			lcd.clear();
			time_on = millis();
			ButtonCCW();
            //EEPROM_readAnything(0, azim.az_cur);
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("Current position: ");
			lcd.setCursor(0,1);
			lcd.print("Azimuth: ");
			lcd.print(azim.az_cur);
			lcd.print(" deg");
		}
        

		//manual CW button pressed

		if (digitalRead(rotButCW) == HIGH) 
		{
            lcd.clear();
			time_on = millis();
			ButtonCW();
			//EEPROM_readAnything(0, azim.az_cur);
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("Current position: ");
			lcd.setCursor(0,1);
			lcd.print("Azimuth: ");
			lcd.print(azim.az_cur);
			lcd.print(" deg");
		}

		//turn off rotor
		digitalWrite(relayEXEpin, LOW);




 // wait for serial input full serial line data (min 12 bytes in buffer)

if (Serial.available() > 12) {

	
    //read the first byte
    startbyte = Serial.read();

    // check for the start of the serial string (65) ASCII (A)
    if (startbyte == 65) 
	
	{    
    
		for (s=0; s<12;s++) 
		{
			predictInput[s] = Serial.read();
			
   		}

				
	char az[5] = { predictInput[1], predictInput[2], predictInput[3] };
	
	// write input value to serial for debugging
	Serial.print("Input: ");
	Serial.println(az);
	
	int az_new = atoi(az);  //Convert azimuth char string to integer
		
	az_new = constrain(az_new, 0, 359); //limit azimuth between 0 and 359 degrees

	// arite AZ value to serial for debugging purposes
	
		Serial.print("AZ NEW: ");
		Serial.println(az_new);
	
// Rotor function


// Calculate direction 
	long az_diff = az_new - azim.az_cur;

	//If az_diff is < 0 then turn CCW
	if (az_diff < 0) { 
		digitalWrite(ledCCWpin,HIGH);
		digitalWrite(relayROTpin,LOW);
	}				

	else digitalWrite(ledCCWpin,LOW);

	//If az_diff > 0 then turn CW
	if (az_diff > 0) {
		digitalWrite(ledCWpin,HIGH);
		digitalWrite(relayROTpin,HIGH);
	}		

	else digitalWrite(ledCWpin,LOW);


// Calculate timing
  
		if (az_diff < 0) az_diff = -az_diff;
		unsigned long exec_time = az_diff * 180UL; 

		//If az_diff > 5 degrees then turn. This can be set to whatever azimuth difference desired
		if (az_diff > 5) {  

	    // Turn rotor
		
		
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("Turning ...");

			digitalWrite(relayEXEpin, HIGH);
			delay(exec_time);
			digitalWrite(relayEXEpin,LOW);
			
			lcd.clear();
			lcd.print("Current position:");
			lcd.setCursor(0,1);
			lcd.print(az_new);
			lcd.print(" deg");
		
			lastexe = (millis() / 1000);
		
		//Set az_cur 
		azim.az_cur = az_new;
		//Write az_cur to EEPROM
		EEPROM_writeAnything(0,azim);
		
	}
		
	}
  }
  
    idle = startidle - lastexe;

    if (idle > 59) 

		{
				lcd.setCursor(0,0);
				lcd.print("K5TRI RotorDuino");
				lcd.setCursor(0,1);
				lcd.print("Current pos: ");
				lcd.print(azim.az_cur);
		}
}




int ButtonCW() 
{

	
		
		while (digitalRead(rotButCW) == HIGH) {
		
	    lcd.setCursor(0,0);
        lcd.print("Manual CW ");
		lcd.setCursor(0,1);
		EEPROM_readAnything(0,azim);
                
			Serial.print("on: ");
			Serial.println(time_on);
			digitalWrite(ledCWpin,HIGH);
			digitalWrite(relayROTpin,HIGH);
			digitalWrite(relayEXEpin, HIGH);
			time_off = millis();
			Serial.print("off: ");
			Serial.println(time_off);
        
				time_diff=time_off-time_on;
				Serial.print("diff: ");
				Serial.println(time_diff);
				b = (int) (time_diff / 180);
				az_temp = azim.az_cur + b;
				Serial.print("az_new: ");
				Serial.println(az_temp);
				lcd.print("Current pos: ");
				lcd.print(az_temp);
				lcd.print("   ");

				lastexe = (millis() / 1000);
               			
	}
	
	        //Prevent current position to be greater than 360 when using manual
	        //button to turn CW
            
		if (az_temp > 360) {(az_temp = 360);}
                        azim.az_cur = az_temp;
                        EEPROM_writeAnything(0,azim);
                      	

				}
				




int ButtonCCW() 

{
		 
			
            while (digitalRead(rotButCCW) == HIGH) {
		
         lcd.setCursor(0,0);
		 lcd.print("Manual CCW ");
		 lcd.setCursor(0,1);
		 EEPROM_readAnything(0,azim);

                Serial.print("on: ");
                Serial.println(time_on);
                digitalWrite(ledCCWpin,HIGH);
				digitalWrite(relayROTpin,LOW);
				digitalWrite(relayEXEpin, HIGH);
				time_off = millis();
				Serial.print("off: ");
                Serial.println(time_off);
					
					time_diff = time_off - time_on;
					Serial.print("diff: ");
					Serial.println(time_diff);
					b = (int) (time_diff / 180);
					az_temp = azim.az_cur - b;
					Serial.print("az_new: ");
					Serial.println(az_temp);
					if (az_temp < 0) {(az_temp = 0);}
					lcd.print("Current pos: ");
					lcd.print(az_temp);
					lcd.print("   ");

					lastexe = (millis() / 1000);
            }
	

           // Prevent current position reading from being negative when using manual
           // buttons to turn CCW
           
			if (az_temp < 0) {(az_temp = 0);}
               azim.az_cur = az_temp;
               EEPROM_writeAnything(0,azim);
                       
				}



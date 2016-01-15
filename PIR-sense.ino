/* 
 * Switches a LED according to the state of the sensors output pin.
 * Determines the beginning and end of continuous motion sequences.
 *
 * The sensor's output pin goes to HIGH if motion is present. Except for the Zilog ePIR, this goes LOW!!
 * However, even if motion is present it goes to LOW from time to time, 
 * which might give the impression no motion is present. 
 * This program deals with this issue by ignoring LOW-phases shorter than a given time, 
 * assuming continuous motion is present during these phases.
 *  
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 4 (pd7) on the Arduino
#define ONE_WIRE_BUS 7
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress Temp;

#define DEBUG 1  // If you want to debug set to 1 to do the Serial prints, msp 15012016

int calibrationTime = 30;        //the time we give the sensor to calibrate (10-60 secs according to the datasheet)
long unsigned int lowIn;   		 //the time when the sensor outputs a low impulse
long unsigned int ldrinterval = millis(); //the interval for ldrvalue
long unsigned int pause = 20000;  //the amount of milliseconds the sensor has to be low before we assume all motion has stopped

boolean lockLow = true;
boolean takeLowTime;  

int pirPin = 4;     //the digital pin connected to the PIR sensor's output, Jeenode port 1
int ledPin = 6;     // Led op Jeenode port 3
int relayPin = 5;   // digital pin waar relais is connected, Jeenode Port 2
int ldrPin = A0;    // LDR on AIO port 1

void setup(){
  #if DEBUG
	  Serial.begin(57600);
  #endif
  pinMode(pirPin, INPUT);       // PIR en LDR als input en LED en Relais als output
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  digitalWrite(pirPin, HIGH);   //internal pull up of PIR pin
  digitalWrite(ldrPin, HIGH);   //internal pull up of LDR pin
  
    #if DEBUG
      Serial.print("calibrating sensor ");
    #endif	
    for(int i = 0; i < calibrationTime; i++){   //give the sensor some time to calibrate
      delay(1000);
    }
    #if DEBUG
		  Serial.println(" done");
		  Serial.println("SENSOR ACTIVE");
	  #endif
    delay(50);
    
Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

  // locate devices on the bus
  //Serial.print("Locating devices...");
  //Serial.print("Found ");
  //Serial.print(sensors.getDeviceCount(), DEC);
  //Serial.println(" devices.");

  // report parasite power requirements
  //Serial.print("Parasite power is: "); 
  //if (sensors.isParasitePowerMode()) Serial.println("ON");
  //else Serial.println("OFF");

  // assign address manually.  the addresses below will need to be changed
  // to valid device addresses on your bus.  device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(DeviceAddress, index);
  //Temp = { 0x28, 0xAC, 0x5E, 0x63, 0x3, 0x0, 0x0, 0xF4 };
  //WWretour   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  // 
  // method 1: by index
  if (!sensors.getAddress(Temp, 0)) Serial.println("Unable to find address for Device 0"); 
  //if (!sensors.getAddress(WWretour, 1)) Serial.println("Unable to find address for Device 1"); 
  //if (!sensors.getAddress(CvIn, 2)) Serial.println("Unable to find address for Device 2"); 
  //if (!sensors.getAddress(CvOut, 3)) Serial.println("Unable to find address for Device 3"); 
  //if (!sensors.getAddress(boilerTemp, 4)) Serial.println("Unable to find address for Device 4"); 
  //if (!oneWire.search(Temp)) Serial.println("Unable to find address for Temp");
  //Serial.print("Device 0 Address: ");
  //printAddress(Temp);
  //Serial.println();
  sensors.setResolution(Temp, 12);  // Resolution to 12 bits

  //Serial.print("Device 0 Resolution: ");
  //Serial.print(sensors.getResolution(Temp), DEC); 
  //Serial.println();
  
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  //Serial.print(" Temp F: ");
  //Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void loop(){

    //Serial.print("Requesting temperatures...");
    sensors.requestTemperatures();
    //Serial.println("DONE");
    //printData(Temp);
    float tempC = sensors.getTempC(Temp);
    //Serial.print("Temp C: ");
    //Serial.print(tempC);
  

     if(digitalRead(pirPin) == HIGH){
       digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
       digitalWrite(relayPin, HIGH);   //relaypin omhoog!
       byte ldrval = analogRead(ldrPin); //lees LDR waarde in byte(dus 0-255).
       float tempC = sensors.getTempC(Temp);
       
       if(lockLow){  
         lockLow = false;    //makes sure we wait for a transition to LOW before any further output is made:
                  
         #if DEBUG
			      Serial.println("---");
			      Serial.print("motion detected at ");
			      Serial.print(millis()/1000);
			      Serial.print(" sec, LDR = "); 
            Serial.print(ldrval);
            Serial.print(" Temp = ");
            Serial.println(tempC);
		     #endif
         
         delay(50);
         }         
         takeLowTime = true;
       }

     if(digitalRead(pirPin) == LOW){       
       digitalWrite(ledPin, LOW);  //the led visualizes the sensors output pin state
              
       if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
        
       if(!lockLow && millis() - lowIn > pause){    //if the sensor is low more than pause, there is no more motion
           lockLow = true;                          //makes sure this code is only executed again when new motion sequence has been detected
           digitalWrite(relayPin, LOW);             //relaypin omlaag!                       
           byte ldrval = analogRead(ldrPin);        //lees LDR waarde in byte(dus 0-255).
           float tempC = sensors.getTempC(Temp);
           
           #if DEBUG
			        Serial.print("motion ended at    ");      //output
			        Serial.print((millis() - pause)/1000);
			        Serial.print(" sec, LDR = "); 
              Serial.print(ldrval);
              Serial.print(" Temp = ");
              Serial.println(tempC);
  
		       #endif
           
           delay(50);
           }
       }
  }

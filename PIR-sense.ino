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

//Defines..... msp, 07012016
#define DEBUG 1  // If you want to debug set to 1 to do the Serial prints

/////////////////////////////
//VARS
int calibrationTime = 30;        //the time we give the sensor to calibrate (10-60 secs according to the datasheet)

long unsigned int lowIn;   		 //the time when the sensor outputs a low impulse
     
long unsigned int pause = 20000;  //the amount of milliseconds the sensor has to be low before we assume all motion has stopped

boolean lockLow = true;
boolean takeLowTime;  

int pirPin = 4;     //the digital pin connected to the PIR sensor's output, Jeenode port 1
int ledPin = 6;     // Led op Jeenode port 3
int relayPin = 5;   // digital pin waar relais is connected, Jeenode Port 2


/////////////////////////////
//SETUP
void setup(){
  #if DEBUG
	  Serial.begin(57600);
  #endif
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(pirPin, HIGH);

  //give the sensor some time to calibrate
  #if DEBUG
    Serial.print("calibrating sensor ");
  #endif	
    for(int i = 0; i < calibrationTime; i++){
      delay(1000);
    }
    #if DEBUG
		  Serial.println(" done");
		  Serial.println("SENSOR ACTIVE");
	  #endif
    delay(50);
  }

////////////////////////////
//LOOP
void loop(){

     if(digitalRead(pirPin) == HIGH){
       digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
       digitalWrite(relayPin, HIGH);   //relaypin omhoog!
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         #if DEBUG
			    Serial.println("---");
			    Serial.print("motion detected at ");
			    Serial.print(millis()/1000);
			    Serial.println(" sec"); 
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
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true; 
           digitalWrite(relayPin, LOW);   //relaypin omlaag!                       
           #if DEBUG
			      Serial.print("motion ended at ");      //output
			      Serial.print((millis() - pause)/1000);
			      Serial.println(" sec");
		       #endif
           delay(50);
           }
       }
  }


/****************************************************************
* Internal Methods:
* long ReadQTI(int pin) - Reads the IR sensor, returns a value
*                        between 0 that indicated the lightness or
*                        darkness under the IR sensor (0=white)
* long ReadSonarInches (int pin) - Reads the sonar and returns 
*                                  a value indicating the distanc
*                                  of an object infront of the 
*                                  robot.
* boolean Obstacle(int pin) - Returns true if there is an obstacle 
*                             2" or less in front of the robot. 
*                             Otherwise, it returns false.
*****************************************************************/

#include <Servo.h> 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define SonarPin 2        // Sonar pin
#define OnBoardLED 13     // On board LED for signalling and debugging
#define RightServo 5      // Right servo pin
#define LeftServo 6       // left servo pin
#define LeftQTIPin A0     // Left IR sensor pin
#define CenterQTIPin A1   // Center IR sensor pin
#define RightQTIPin A2    // Right IR sensor pin
#define ServoStop 90      // PWM value to stop the servos
//----------------- Parallax Servos---------------------------------------
#define CWSFull 75         // PWM value for clockwise servo motion - High Speed 
#define CCWSFull 105      // PWM value for counter clockwise servo motion - High Speed 
#define CWSMid 83         // PWM value for clockwise servo motion - Mid Speed
#define CCWSMid 97      // PWM value for counter clockwise servo motion - Mid Speed
#define CWSSlow 89        // PWM value for clockwise servo motion - Slow Speed
#define CCWSSlow 91       // PWM value for counter clockwise servo motion - Slow Speed

#define RWOffSet  0       // Right and left servo velocity offsets. Compensates for 
#define LWOffSet  0       // differences in servos velocities that can't be fixed in calibration.
                          // Helps bots maintain straight line of travel.

#define Threshold 100     // This is the threshold from white to black. You will have to adjust this 
                          // for each bot based on tests performed with the QTI tests. Any values less
                          // than Threshold will be considered white. Above Threshold, black.

#define Cthreshold 110  
#define Rthreshold 125  
#define Lthreshold 100  
#define servoHalt 90

// Parameters

int leftQti;              // Left IR sensor value
int centerQti;            // Center IR sensor value
int rightQti;             // Right IR sensor value

Servo leftservo;          // Left servo object
Servo rightservo;         // Right servo object

RF24 radio(7, 8);                 // Chip enable (7), Chip Select (8)
const byte address[6] = "01011";  // Radio address - use only the channels that match the
                                  // numbers on your robots.
//

 // Added new character arrays 
static int flag;
static int shiprec;
static int stopflag;
static int initialflag; 
const char text1[] = "L";
const char text2[] = "C";
const char text3[] = "R";
const char centered[]="centered";
const char jogleft[]="joggingleft";
const char jogright[]="joggingright";
const char centered1[]="centered1";
const char jogleft1[]="joggingleft1";
const char jogright1[]="joggingright1";
const char receiving[]="receiving";
const char shipping[]="shipping";
const char obstacle[]="obstacle";
char lqt[16];
char cqt[16];
char rqt[16];
char msg[200];
const char colon1[]=":";
const char colon2[]="||";
char flagg[10];

 
void setup() 
{ 
  // Initialize the Serial port:
  Serial.begin(9600);
  leftservo.attach(LeftServo);
  rightservo.attach(RightServo);
  leftservo.write(ServoStop); 
  rightservo.write(ServoStop);

  pinMode(OnBoardLED, OUTPUT);

  radio.begin();                  // Instantiate the radio object
    
  if (!radio.isChipConnected())
  {
      Serial.print("RADIO NOT CONNECTED!");
      while(!radio.isChipConnected());
  }

   radio.openReadingPipe(0, address);  // Open the radio pipe using your address (read about pipes and channels)
  radio.setPALevel(RF24_PA_MIN);      // Set the power level. Since the bots and the radio base station are close I use min power
  radio.startListening();             // Go into receive mode.
  Serial.println("Radio Ready...");

  delay(5000);
} 

void loop() 
{

    // Read the QTI sensors
    leftQti = ReadQTI(LeftQTIPin);
    centerQti = ReadQTI(CenterQTIPin);
    rightQti = ReadQTI(RightQTIPin);

//    itoa(leftQti, lqt, 10);
//    itoa(centerQti, cqt, 10);
//    itoa(rightQti, rqt, 10);
//    itoa(flag,flagg,10);
//    radio.write(&flagg,sizeof(flagg));


    
    // These are debug messages - obviously not printed when untethered
//    Serial.print("Left QTI: ");
//    Serial.print(leftQti);      // Displays results of left QTI
//    Serial.print("  Center QTI: ");
//    Serial.print(centerQti);    // Displays results of center QTI
//    Serial.print("  Right QTI: ");  
//    Serial.println(rightQti);   // Displays results of right QTI
//
//    //Write it to the radio station 
//    strcpy(msg,text1);
//    strcat(msg,colon1);
//    strcat(msg,lqt);
//    strcat(msg,colon2);
//    strcat(msg,text2);
//    strcat(msg,colon1);
//    strcat(msg,cqt);
//    strcat(msg,colon2);
//    strcat(msg,text3);
//    strcat(msg,colon1);
//    strcat(msg,rqt);    
//    radio.write(&text1, sizeof(text1));
//    radio.write(&lqt,sizeof(lqt));
//    radio.write(&text2, sizeof(text2));
//    radio.write(&cqt,sizeof(cqt));
//    radio.write(&text3, sizeof(text3));
//    radio.write(&rqt,sizeof(rqt));

    // In this section we check the values of the Sonar and the QTI pins
    // and figure out what to do.
    if (radio.available())              // If we have messages, we print them out - otherwise we do nothing but listen.
    {
      char text[32] = "";
      radio.read(&text, sizeof(text));
      if(!strcmp(text,"m"))
      {
        Serial.println("1");
        Serial.print(text);
        leftservo.write(servoHalt);
        rightservo.write(servoHalt);
        while(1)
        {
           char text[32] = "";
           radio.read(&text, sizeof(text));
           if(!strcmp(text,"n"))
           {
              break;
           }
         }
      }
    }
    if (Obstacle(SonarPin)){
      // Some obstacle is in front of the robot (within 2 inches)
      Serial.print("Obstacle!");
      leftservo.write(ServoStop); 
      rightservo.write(ServoStop);
//      radio.write(&obstacle,sizeof(obstacle));
//      radio.write(&msg,sizeof(msg));  

      
    } else if ((leftQti <Lthreshold) && (centerQti>Cthreshold) && (rightQti<Rthreshold)) {
      // centered on the line
      leftservo.write(CCWSFull+LWOffSet); 
      rightservo.write(CWSFull+RWOffSet);
//      Serial.println( "centered" );
//      radio.write(&centered,sizeof(centered));
//      radio.write(&msg,sizeof(msg));

      
    } else  if ((leftQti>Lthreshold) && (centerQti<Cthreshold) && (rightQti<Rthreshold)) {
      // Drifted right
      leftservo.write(CCWSSlow); 
      rightservo.write(CWSMid);
      Serial.println( "jog left" );
//      radio.write(&jogleft,sizeof(jogleft));
      flag=1;
//    radio.write(&msg,sizeof(msg));

    
    } else if ((leftQti<Lthreshold) && (centerQti<Cthreshold) && (rightQti>Rthreshold)) {

      // Drifted left
      leftservo.write(CCWSMid); 
      rightservo.write(CWSSlow);
      Serial.println( "jog right" );      
//      radio.write(&jogright,sizeof(jogright));
      flag=2;
//          radio.write(&msg,sizeof(msg));

          
    } else if ((leftQti>Lthreshold) && (centerQti>Cthreshold) && (rightQti>Rthreshold) && (shiprec==2 || initialflag==0)) {
      // At shipping
      Serial.println( "shipping" ); 
//      radio.write(&shipping,sizeof(shipping));
      leftservo.write(ServoStop); 
      rightservo.write(ServoStop);      
      delay(2000);
      while(1)
      {
        char text1[32] = "";
        radio.read(&text1, sizeof(text1));
        if(!strcmp(text1,"m"))
        {
          leftservo.write(servoHalt);
        rightservo.write(servoHalt);
        while(1)
        {
           char text[32] = "";
           radio.read(&text, sizeof(text));
           if(!strcmp(text,"n"))
           {
              break;
           }
         }
        }
        else if(!strcmp(text1,"q"))
        {
          leftservo.write(CCWSMid+LWOffSet); 
          rightservo.write(CWSMid+RWOffSet);
                delay(1000);
          break;
        }
      }
//      delay(1000);
      shiprec=0;
      stopflag=1;
      initialflag=1;

          
    } else if ((leftQti>Lthreshold) && (centerQti>Cthreshold) && (rightQti>Rthreshold) && shiprec!=2) {
      
     // centered on the line
      leftservo.write(CCWSFull+LWOffSet); 
      rightservo.write(CWSFull+RWOffSet);
      Serial.println( "centered" );

          
    }else if ((leftQti>Lthreshold) && (centerQti<Cthreshold) && (rightQti>Rthreshold) &&shiprec==0) {
      // At receiving
      Serial.println( "receiving" ); 
//      radio.write(&receiving,sizeof(receiving));
      leftservo.write(ServoStop); 
      rightservo.write(ServoStop);
      delay(2000);
      
      
      while(1)
      {
        
        char text2[32] = "";
        radio.read(&text2, sizeof(text2));
        if(!strcmp(text2,"m"))
        {
         leftservo.write(servoHalt);
        rightservo.write(servoHalt);
        while(1)
        {
           char text[32] = "";
           radio.read(&text, sizeof(text));
           if(!strcmp(text,"n"))
           {
              break;
           }
         }
        }
        else if(!strcmp(text2,"q"))
        {
          leftservo.write(CCWSMid+LWOffSet); 
          rightservo.write(CWSMid+LWOffSet);
          delay(1000);
          break;
        }
      }
      stopflag=1;
      shiprec=2;

          
//Extra loop 
    } else if((leftQti<Lthreshold) && (centerQti<Cthreshold) && (rightQti<Rthreshold)) {
      
      
      if(flag==1){
      // Drifted right
      leftservo.write(CCWSSlow); 
      rightservo.write(CWSMid);
      Serial.println( "jog left" );
//      radio.write(&jogleft1,sizeof(jogleft1));
//      radio.write(&msg,sizeof(msg)); 

      
      }
      else if(flag==2){
      // Drifted left
      leftservo.write(CCWSMid); 
      rightservo.write(CWSSlow);
      Serial.println( "jog right" );      
//      radio.write(&jogright1,sizeof(jogright1)); 
//          radio.write(&msg,sizeof(msg)); 

          
      }else if(flag==0){
      leftservo.write(CCWSFull+LWOffSet); 
      rightservo.write(CWSFull+RWOffSet);
      Serial.println( "centered" );
//      radio.write(&centered1,sizeof(centered1));
//          radio.write(&msg,sizeof(msg));

          
      }
    
    }else if((leftQti<Lthreshold) && (centerQti>Cthreshold) && (rightQti>Rthreshold)){
      // Drifted right
      leftservo.write(CCWSSlow); 
      rightservo.write(CWSMid);
      Serial.println( "jog left" );
//      radio.write(&jogleft1,sizeof(jogleft1));
      flag=2;
//          radio.write(&msg,sizeof(msg));

          
    }else if((leftQti>Lthreshold) && (centerQti>Cthreshold) && (rightQti<Rthreshold)){
      // Drifted left
      leftservo.write(CCWSMid); 
      rightservo.write(CWSSlow);
      Serial.println( "jog right" );      
//      radio.write(&jogright1,sizeof(jogright1));
      flag=1;
//          radio.write(&msg,sizeof(msg));

          
    }
    
    
} // loop

/****************************************************************
* ReadQTI (int pin)
* 
* Parameters:
*              int pin - the pin on the Arduino where the QTI
*                        sensor is connected.
* Description:
*  - Reads the shade of the area under the QTI
* sensor by determines how long it takes an on-board capactior
* to charge/discharge. This method initalizes the sensor pin as  
* output and charges the capacitor on the QTI. The QTI emits IR 
* light which is reflected off of any surface in front of the 
* sensor. The amount of IR light reflected back is detected by 
* the IR resistor on the QTI. This is the resistor that the 
* capacitor discharges through. The amount of time it takes to 
* discharge determines how much light, and therefore the 
* lightness or darkness of the material in front of the QTI 
* sensor.
* 
* Zero or lower single digit values indicate white
* Higher values indicate dark
****************************************************************/

long ReadQTI(int sensorIn)
{
  long duration = 0;
  pinMode(sensorIn, OUTPUT);          // Sets pin as OUTPUT
  digitalWrite(sensorIn, HIGH);       // Pin HIGH
  delay(1);                           // Allow capacitor to fully discharge
  pinMode(sensorIn, INPUT);           // Sets pin as INPUT
  digitalWrite(sensorIn, LOW);        // Pin LOW
  while (digitalRead(sensorIn))
    duration++;                       // LOW (cap discharges)

  return duration;                    // Returns the duration of the pulse
}

/****************************************************************
* boolean Obstacle (int pin) 
* 
* Parameters: None
*              int pin - the pin on the Arduino where the sonar
*                        is connected.          
* Description:
* Issues a command to the sonar checks for objects in front of the 
* robot. If there is an obstacal within 2 inches this method will
* return true, otherwise it will return false.
****************************************************************/
boolean Obstacle( int pin )
{
  if (ReadSonarInches(SonarPin) <= 4 )
  {
    Serial.println(true);
    return( true );
    
  } else {   
    Serial.println(false); 
    return( false );  
  }
  
}
    
/****************************************************************
* ReadSonarInches (int pin) 
* 
* Parameters:
*              int pin - the pin on the Arduino where the sonar
*                        is connected.
* Description:
* Issues a command for the sonar to perform a ping-echo to 
* determine the distance of any objects in front of the sonar. A
* pulse triggers the ping, then we count how long it takes to 
* receive the return echo. The ping))) sonar will measure between
* 1 inch and 144 inches. This routine returns distance in inches.
*
****************************************************************/

long ReadSonarInches( int pin )
{

  long duration;  // Duration of the ping/echo
  long inches;    // Distance in inches

  
  // The sonar is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the sonar. A HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);
   
  // Sound travels at 1130 feet per second, so there are 73.746 microseconds per inch. 
  // This gives the distance travelled by the ping, outbound and return, so we divide 
  // by 2 to get the distance to the obstacle.
  
  if (duration/74/2 == 0 )
  { 
    digitalWrite(OnBoardLED, HIGH);
    delay(500);
    digitalWrite(OnBoardLED, LOW);
  }
  return duration/74/2;
}

 

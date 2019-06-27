#include <Servo.h> 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//------------------------------ Pins for senors----------------------------------//
#define SonarPin 2        // Sonar pin
#define OnBoardLED 13     // On board LED for signalling and debugging
#define RightServo 5      // Right servo pin
#define LeftServo 6       // left servo pin
#define LeftQTIPin A0     // Left IR sensor pin
#define CenterQTIPin A1   // Center IR sensor pin
#define RightQTIPin A2    // Right IR sensor pin
#define ServoStop 90      // PWM value to stop the servos
//--------------------------------------------------------------------------------//


//------------------------------ Parallax Servos---------------------------------------//
#define CWSFull 75         // PWM value for clockwise servo motion - High Speed 
#define CCWSFull 105      // PWM value for counter clockwise servo motion - High Speed 
#define CWSMid 83         // PWM value for clockwise servo motion - Mid Speed
#define CCWSMid 97      // PWM value for counter clockwise servo motion - Mid Speed
#define CWSSlow 75        // PWM value for clockwise servo motion - Slow Speed
#define CCWSSlow 105       // PWM value for counter clockwise servo motion - Slow Speed
//-------------------------------------------------------------------------------------//

//----Thresholds for QTI for bang bang----//
#define Cthreshold 110  
#define Rthreshold 125  
#define Lthreshold 100  
#define servoHalt 90
//-------------------------//

//-----------------QTI reading variables--------------------//
int leftQti;              // Left IR sensor value
int centerQti;            // Center IR sensor value
int rightQti;             // Right IR sensor value
//---------------------------------------------------------//

//--------------------Address, Servo and radio variables------------------//
Servo leftservo;          // Left servo object
Servo rightservo;         // Right servo object
RF24 radio(7, 8);                 // Chip enable (7), Chip Select (8)
const byte address[6] = "01011";  // Radio address - use only the channels 
                                  //that match the numbers on your robots
//------------------------------------------------------------------------//


//---characters for sending messages back---//
const char receiving[]="receiving";
const char shipping[]="shipping";
const char obstacle[]="obstacle";
char lqt[16];
char cqt[16];
char rqt[16];
char msg[200];
const char colon1[]=",";
const char colon2[]="\n";
//-----------------------------------------//


//---------------Constans for PID ------------------//
#define KP 0.015 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 0.09 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define KI 0.00005 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Ki)

//------------------------------------------------//




void setup() {
  
  // Initialize the Serial port and attach servos 
  Serial.begin(9600);
  leftservo.attach(LeftServo);
  rightservo.attach(RightServo);
  leftservo.write(ServoStop); 
  rightservo.write(ServoStop);

  pinMode(OnBoardLED, OUTPUT);

  // Instantiate the radio object
  radio.begin();                  
    
  if (!radio.isChipConnected())
  {
      Serial.print("RADIO NOT CONNECTED!");
      while(!radio.isChipConnected());
  }

  //Opening radio pipe 
  radio.openWritingPipe(address);  // Open the radio pipe using your address (read about pipes and channels)
  radio.setPALevel(RF24_PA_MIN);   // Set the power level. Since the bots and the radio base station are close I use min power
  radio.stopListening();           // Now we listen for messages...
  Serial.println("Radio Ready...");

  delay(1000);

}

void loop() {

    static int errorsum;
    static int lasterror;
     // Read the QTI sensors
    leftQti = ReadQTI(LeftQTIPin);
    centerQti = ReadQTI(CenterQTIPin);
    rightQti = ReadQTI(RightQTIPin);

    itoa(leftQti, lqt, 10);
    itoa(centerQti, cqt, 10);
    itoa(rightQti, rqt, 10);
    
    //Write it to the radio station 
    strcpy(msg,lqt);
    strcat(msg,colon1);
    strcat(msg,cqt);
    strcat(msg,colon1);
    strcat(msg,rqt);  
    strcat(msg,colon2);


    // In this section we check the values of the Sonar and the QTI pins
    // and figure out what to do. Some obstacle is in front of the robot 
    // (within 2 inches)

    if (Obstacle(SonarPin)){
      Serial.print("Obstacle!");
      leftservo.write(ServoStop); 
      rightservo.write(ServoStop);
      radio.write(&msg,sizeof(msg));  

    }
    else
    {

      int position = FindPosition(leftQti,centerQti,rightQti);
      int error = position - 1000;

      //Integral part. Value of KI is defined above
//      errorsum+=error;
      
//      this is the D part of PID, we will do that later. 
//      int motorSpeed = KP * error + KD * (error - lastError);
      lasterror = error;

      //The motorspeed 
      int motorSpeed = KP * error + KD * (error - lasterror) ;  

      int leftMotorSpeed = CCWSSlow + motorSpeed;
      int rightMotorSpeed = CWSSlow + motorSpeed;    

      //This block to ensure that speed doesn't go below or above the low
      //and high speed 
      if(rightMotorSpeed>=90)
      {
        rightMotorSpeed = 89;
      }
      if(rightMotorSpeed<0)
      {
        rightMotorSpeed=0;
      }
      if(leftMotorSpeed<=90)
      {
        leftMotorSpeed = 91;
      }
      if(leftMotorSpeed>180)
      {
        leftMotorSpeed=180;
      }
      
       
      //Set motor speeds using the two motor speed variables above
      leftservo.write(leftMotorSpeed); 
      rightservo.write(rightMotorSpeed);
      radio.write(&msg,sizeof(msg));
    }
      
}


/****************************************************************
* FindPosition (long leftQti, long centerQti, long rightQti)
* 
* Parameters:
*              long leftQti - The left IR sensor reading .
*              long centerQti - The center IR sensor reading .
*              long rightQti - The right IR sensor reading .

* Description:
*  Combines the Qti sensor values to calculate the position of the
*  w.r.t to the black line. If the value is close to 1000, it 
*  implies that the bot is close to the line, if the value is
*  closer to zero means that the bot is on the left side of the 
*  line and if the value is close to 2000, then the bot is 
*  on the right side of the line. 
* Returns : It returns an integer value of the error. 
****************************************************************/
int FindPosition(long leftQti, long centerQti, long rightQti)
{

//This is inspired from the QTR sensor's package of linefollow demo
// Please refer to https://www.pololu.com/docs/0J19/3 for more info
  long numerator = (-50*leftQti) + (1000*centerQti) + (2000*rightQti); 
  long denominator = (leftQti) + (centerQti) + (rightQti);
  long error = numerator / denominator;
  int err= error;
  
  return err; 
}


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


/***************************************************************************

 
 */

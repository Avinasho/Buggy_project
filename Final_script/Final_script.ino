/* 
 *  Code to control a line-following buggy using an arduino board.
 *  Final version completed 21 January 2017
 *  Written by Avinash  Soor
 *  Git: Avinasho
 *  Written for a mechatronics project
*/

#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
/* These first two '#include' lines allow the adafruit motorshield to allow coding
   in the correct syntnax, so that the motors can be run. */

// Pre-allocation and assigning of constant integers.
const int TriggerPin = 5;                           // 'Trig' pin, attached to digital pin 5.
                                                    // The trigger pulse input pin.
const int EchoPin = 6;                              // 'Echo' pin, attached to digital pin 6.
                                                    // The ceho pulse output pin.
const int S1 = 7;                                   // Line sensor 1, attached to digital pin 7.
const int S2 = 8;                                   // Line sensor 2, attached to digital pin 8.

Adafruit_MotorShield a_ms = Adafruit_MotorShield(); // Create the motor shield obbject
                                                    // with the default I2C address.
Adafruit_DCMotor *myMotor1 = a_ms.getMotor(1);      // Motor 1, (the right wheel), on port M1
                                                    // of the adfruit motorshield.
Adafruit_DCMotor *myMotor2 = a_ms.getMotor(2);      // Motor 2, (the left wheel), on port M2
                                                    // of the adfruit motorshield.
                                                    
int S1_State = 0;                                   // Initial sensor 1 value.
int S2_State = 0;                                   // Initial sensor 2 value.
long Duration = 0;                                  // Initial duration

void setup() 
{
  pinMode(S1, INPUT);                              // Setting sensor 1 as an input.
  pinMode(S2, INPUT);                              // Setting sensor 2 as an input.

  pinMode(TriggerPin,OUTPUT);                      // Setting the trigger pin as an output.
  pinMode(EchoPin,INPUT);                          // Setting the echo pin as an input.

  a_ms.begin();                                    // Initialises the adafruit motorshield.
  
  Serial.begin(9600);                              // Sets the baud for serial data transmission.
}

long Distance(long time)
/* Calculates the Distance in front of the ultrasonic sensor in mm.
   ((time)*(Speed of sound))/back and forth of signal) * 10 */
{ 
  long DistanceCalc;                               // Calculation variable.
  DistanceCalc = ((time /2.9) / 2);                // Actual calculation in mm.
  return DistanceCalc;                             // Return calculated value.
}

void loop()
{
  S1_State = digitalRead(S1);                      // Reads the state of S1 and assigns it to S1_State.
  S2_State = digitalRead(S2);                      // Reads the state of S2 and assigns it to S2_State.

  Serial.print("S1: ");
  Serial.println(S1_State);                        // Prints the current s2 value to the serial monitor.
  Serial.print("S2: ");
  Serial.println(S2_State);                        // Prints the current s2 value to the serial monitor.
  
  digitalWrite(TriggerPin, LOW);                   // Trigger pin set to LOW.
  delayMicroseconds(2);                            // Delays by 2us.
  digitalWrite(TriggerPin, HIGH);                  // Trigger pin set to HIGH.
  delayMicroseconds(10);                           // Delays by 10us.
  digitalWrite(TriggerPin, LOW);                   // Trigger pin set to LOW.
  Duration = pulseIn(EchoPin,HIGH);                // Waits for the echo pin to get high.
  long distance = Distance(Duration);              // Use function to calculate the distance.
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" mm");                           // Outputs distance to serial monitor.
  
  if( distance >= 100 )
  /* If there nothing blocking the sensor within 10cm, run the motors. */
  {
    if( (S1_State == 1) && (S2_State == 1) )       // If both sensors read a 1, (black).
    {
      myMotor1->setSpeed(30);
      myMotor1->run(BACKWARD);
      myMotor2->setSpeed(30);
      myMotor2->run(BACKWARD);
      // Both motors are set to run backwards at a speed of 30, in an attempt to auto-correct.
    }
    if( (S1_State == 0) && (S2_State == 1) )       // If S1 reads a 0, and S2 a 1.
    {
      myMotor1->setSpeed(90);
      myMotor1->run(BACKWARD);
      myMotor2->setSpeed(105);
      myMotor2->run(FORWARD);
      // The right motor goes backwards at 90, and the left forward at 105, to turn right.
    }
    if( (S1_State == 1) && (S2_State == 0) )       // If S1 reads a 1, and S2 a 0.
    {
      myMotor1->setSpeed(105);
      myMotor1->run(FORWARD);
      myMotor2->setSpeed(90);
      myMotor2->run(BACKWARD);
      // The right motor goes backwards at 105, and the left forward at 95, to turn left.
    }
    if( (S1_State == 0) && (S2_State == 0) )       // If both sensors read a 0, (white).
    {
      myMotor1->setSpeed(100);
      myMotor1->run(FORWARD);
      myMotor2->setSpeed(100);
      myMotor2->run(FORWARD);
      // Both motors are set to run forwards at a speed of 100, to move in a straight line.
    }
  }
  else if( distance < 100 )
  /* If there is something up to 10 cm in front of the sensor, the motors stop. */
  {
      myMotor1->setSpeed(0);
      myMotor1->run(BACKWARD);
      myMotor2->setSpeed(0);
      myMotor2->run(BACKWARD);
  }
  delay(55);                                       // Delays running the loop again by 55ms.
}

/*  
 *
 *     ********************************************************
 *     ********************************************************
 *     ***                                                  ***
 *     ***                Pollywog Droid                   ***
 *     ***                                                  ***
 *     ******************************************************** 
 *     ********************************************************
 *
 *    Arduino code for the Pollywog robot for using with the
 *    Android RoboPad app. 
 *     
 *   ****************************************************
 *   * Fecha: 05/03/2014                                *
 *   * Autor:Estefana Sarasola Elvira                   *
 *   * Mail: diy@bq.com                                 *
 *   * Licencia: GNU General Public License v3 or later *
 *   ****************************************************
 */

/******************************************************************/
/******************************************************************/




/******************************************************************
 *                           Libraries                            *
 ******************************************************************/ 

#include <SoftwareSerial.h>


/******************************************************************
 *                    Definition of variables                     *
 ******************************************************************/
/* Pines para SoftwareSerial (plugin de GGUU) */
#define BTRX 7
#define BTTX 5

SoftwareSerial BTSerial(BTRX, BTTX); 

/* Pin definition of the board to be used */
#define pinLeftWheelFordward    3
#define pinLeftWheelBackward    6
#define pinRightWheelFordward   10
#define pinRightWheelBackward   11

#define pinSensorIRLeft         8   /*   Left infrared sensor     */ 
#define pinSensorIRRight        9   /*   Right infrared sensor    */

/* Define the posible states of the state machine of the program */
#define MANUAL_CONTROL_STATE    0
#define LINE_FOLLOWER           1

/* Baud rate of the Bluetooth*/
#define BLUETOOTH_BAUDRATE    38400

/* Definition of the pwm values that can take motors for calibrated movement */
#define leftWheelFordwardValue    255
#define leftWheelBackwardsValue   255
#define rightWheelFordwardValue   255
#define rightWheelBackwardsValue  255

/* Size of the received data buffer */
#define bufferSize 1

/* Default delay */
#define defaultDelay        10

/* Variable that controls the current state of the program */
int currentState;

/* Variables of the line follower mode */
int rightIR;
int leftIR;
int BLACK = 0;
int WHITE = 1;

/*  A char buffer to storage the received data from the Bluetooth
    Serial */
char dataBuffer[bufferSize]; 

/* Buffer iterator */
int i = 0;

/* Number of characters availables in the Serial */
int numChar = 0;


/******************************************************************
 *                     Definition of functions                    *
 ******************************************************************/

void stopWheels() {
  digitalWrite(pinLeftWheelFordward, 0);
  digitalWrite(pinLeftWheelBackward, 0);
  digitalWrite(pinRightWheelFordward, 0);
  digitalWrite(pinRightWheelBackward, 0);
  delay(defaultDelay);
}

void goForwards() {
  digitalWrite(pinLeftWheelFordward, leftWheelFordwardValue);
  digitalWrite(pinLeftWheelBackward, 0);
  digitalWrite(pinRightWheelFordward, rightWheelFordwardValue);
  digitalWrite(pinRightWheelBackward, 0);
  delay(defaultDelay);
}

void goBackwards() {
  digitalWrite(pinLeftWheelFordward, 0);
  digitalWrite(pinLeftWheelBackward, leftWheelBackwardsValue);
  digitalWrite(pinRightWheelFordward, 0);
  digitalWrite(pinRightWheelBackward, rightWheelBackwardsValue);
  delay(defaultDelay);
}

void goLeft() {
  digitalWrite(pinLeftWheelFordward, 0);
  digitalWrite(pinLeftWheelBackward, leftWheelBackwardsValue);
  digitalWrite(pinRightWheelFordward, rightWheelFordwardValue);
  digitalWrite(pinRightWheelBackward, 0);
  delay(defaultDelay);
}

void goRight() {
  digitalWrite(pinLeftWheelFordward, leftWheelFordwardValue);
  digitalWrite(pinLeftWheelBackward, 0);
  digitalWrite(pinRightWheelFordward, 0);
  digitalWrite(pinRightWheelBackward, rightWheelBackwardsValue);
  delay(defaultDelay);
}

/*
  Perform the action required by the user of the Android app
*/
void setAction(char* data) {
  
  switch(data[0]) {

    /* Line follower mode button pressed */
    case 'I':
      currentState = LINE_FOLLOWER;
      break;

    /* Manual control mode button pressed */
    case 'M':
      currentState = MANUAL_CONTROL_STATE;
      stopWheels();
      break;
   
    /* Stop button pressed */
    case 'S':
      stopWheels();
      break;

    /* Up button pressed  */
    case 'U':
      goForwards();
      break;

    /* Down button pressed  */ 
    case 'D':
      goBackwards();
      break;

    /* Left button pressed  */
    case 'L':
      goLeft();
      break;

    /* Right button pressed  */
    case 'R':
      goRight();
      break;
   
  }
    
  /* Empty the Serial */      
  BTSerial.flush();
    
}


void followTheLine() {
  /* Read the state of the sensors */
  rightIR = digitalRead(pinSensorIRLeft);
  leftIR = digitalRead(pinSensorIRRight);

  /* If the right sensor reads black, we go straight forward, else
     if it reads white, we turn to the left */
  if (rightIR == BLACK) {
    digitalWrite(pinLeftWheelFordward, leftWheelFordwardValue);
    digitalWrite(pinLeftWheelBackward, 0);
    delay(defaultDelay);
  
  } else {
    digitalWrite(pinLeftWheelFordward, 0);
    digitalWrite(pinLeftWheelBackward, 0);
    delay(defaultDelay);
  }
  
  /* If the left sensor reads black, we go straight forward, else
    if it reads white, we turn to the right */
  if (leftIR == BLACK) {
    digitalWrite(pinRightWheelFordward, rightWheelFordwardValue);
    digitalWrite(pinRightWheelBackward, 0);
    delay(defaultDelay);
  
  } else {
    digitalWrite(pinRightWheelFordward, 0);
    digitalWrite(pinRightWheelBackward, 0);
    delay((defaultDelay));
  }
}


/******************************************************************
 *                             Setup                              *
 ******************************************************************/

void setup(){
  
  /* Open the Bluetooth Serial and empty it */
  
  BTSerial.begin(BLUETOOTH_BAUDRATE); 
  BTSerial.flush();     

  /* The robot is stopped at the beginning */
  stopWheels();

  /* Put the wheel signals as output */
  pinMode(pinLeftWheelFordward, OUTPUT);
  pinMode(pinLeftWheelBackward, OUTPUT);
  pinMode(pinRightWheelFordward, OUTPUT);
  pinMode(pinRightWheelBackward, OUTPUT);

  /* Put the IR sensors as input */
  pinMode(pinSensorIRLeft, INPUT);
  pinMode(pinSensorIRRight, INPUT);

  /* Default state is manual control */
  currentState = MANUAL_CONTROL_STATE;
}


/******************************************************************
 *                       Main program loop                        *
 ******************************************************************/

void loop() {
 
   /* If there is something in the Bluetooth serial port */
  if (BTSerial.available() > 0) { 
   
    /* Reset the iterator and clear the buffer */
    i = 0;
    memset(dataBuffer, 0, sizeof(dataBuffer));  
    
    /* Wait for let the buffer fills up. Depends on the length of 
       the data, 1 ms for each character more or less */
    delay(bufferSize); 

    /* Number of characters availables in the Bluetooth Serial */
    numChar = BTSerial.available();   
    
    /* Limit the number of characters that will be read from the
       Serial to avoid reading more than the size of the buffer */
    if (numChar > bufferSize) {
          numChar = bufferSize; 
    }

    /* Read the Bluetooth Serial and store it in the buffer*/
    while (numChar--) {
        dataBuffer[i++] = BTSerial.read();

        /* As data trickles in from your serial port you are 
         grabbing as much as you can, but then when it runs out 
         (as it will after a few bytes because the processor is 
         much faster than a 9600 baud device) you exit loop, which
         then restarts, and resets i to zero, and someChar to an 
         empty array.So please be sure to keep this delay */
        delay(3);
    } 

    /* Manage the data */
    setAction(dataBuffer);
    
  }

  if(currentState == LINE_FOLLOWER) {
    followTheLine();
  }

}  
  
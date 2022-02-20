/*
  DB1 Robot Project - Motor Controller Test
  db1-motor-control-test.ino
  Controls single DC Gearmotor with Rotary Encoder
  Motor driven with Cytron MD10C Motor Driver
  Basic Emergency Stop Functionality Test
  Emergency Stop on INT1
  Version 0.43
  Updated 2020-04-30
  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/


#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>



// Robot-Specific Options
// Change as required

// Motor Parameters (change as required)
// Motor rotary encoder output pulses per rotation
float count_rev = 384.5;

// Wheel Parameters
// Wheel diameter in millimeters, change if different
float wheel_diameter = 101.60;


// Pin number definitions
// Arduino Nano or ATMega328 design

// Rotary Encoder output to Arduino Interrupt pin 2 (ATMega328 pin 4) - INT 0
#define ENC_IN 36 //TODO

// Emergency Stop to Arduino Interrupt pin 3 (ATMega328 pin 5) - INT 1
#define EM_STOP 23 //TODO

// MD10C PWM connected to esp32 32 ADC1 ch4
#define MD10C_PWM 32

// MD10C DIR connected to esp32 33 ()
#define MD10C_DIR 33

/*
// I2C Address Select connected to Arduino pins 4 & 5 (ATMega328 pins 6 & 11)
#define I2C_ADDR0 4
#define I2C_ADDR1 5

// PWM Frequency Select connected to Arduino pin 6 (ATMega328 pin 12)
#define PWM_FREQ 6
*/
// Status LED Output connected to Arduino pin 13 (ATMega328 pin 19)
#define LED_STATUS 4  //turn right
#define LED_LEFT 13

//left right pin
#define LR_PIN 34

/*
// Processor Status Output connected to Arduino pin 7 (ATMega328 pin 13)
#define PROC_STATUS 7*/

// Emergency Stop LED Output connected to esp32 pin 10 (ATMega328 pin 16)
#define LED_EM_STOP 2

// Emergency Stop Inputs (Analog Inputs used as Digital Inputs)
#define EM_STOP_0 22   // Arduino pin 8 (ATMega328 pin 14)
#define EM_STOP_1 21   // Arduino pin 9 (ATMega328 pin 15)   
#define EM_STOP_2 19  // Arduino pin A0 (ATMega328 pin 23)
#define EM_STOP_3 18  // Arduino pin A1 (ATMega328 pin 24)
#define EM_STOP_4 17  // Arduino pin A2 (ATMega328 pin 25)
#define EM_STOP_5 16  // Arduino pin A3 (ATMega328 pin 26)


// setting PWM properties
const int freq = 5000;
const int MD10C_PWM_channel = 0;
const int resolution = 8;

// Operational Variables

// Pulse count from encoder
volatile long encoderValue = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measurement
int rpm = 0;

// Variable for PWM motor speed output
volatile int motorPwm = 0;

// Variable for motor direction
int motorDir = 1;

// Variable for Motor Status
// 0 = Busy
// 1 = Ready
// 8 = Stop
// 9 = Emergency Stop
volatile int motorStatus = 0;

// Variable for I2C final address
// using esp now
//int i2cAddr;

int check_rng(int input, int max_, int min_){
  if (input < min_) return min_;
  if (input > max_) return max_;
  return input;
}

int max_speed = 255;
int min_speed = 0;
int max_dcl = 60000;
int min_dcl = 0;
int min_tm = 0;
int max_tm = 6000;
int per_default = 1000;
int max_dir = 1;
int min_dir = 0;
int global_tm = min_tm;

int check_rng_speed(int input){
  return check_rng(input, max_speed, min_speed);
}

int check_rng_dcl(int input){
  return check_rng(input, max_dcl, min_dcl);
}

int check_rng_dir(int input){
  return check_rng(input, max_dir, min_dir);
}

int check_rng_tm(int input){
//  Serial.println(input);
//  Serial.println(max_tm);
//  Serial.println(min_tm);
  return check_rng(input, max_tm, min_tm);
}

constexpr char WIFI_SSID[] = "****";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

typedef struct data_struct {
  String sliderValue = "000";
  String variable;
} data_struct;

data_struct myData;

//initialise

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  String variable;
  //Serial.print("Bytes received: ");
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("data recieved: ");
  Serial.println(myData.variable);
  Serial.println();
  motorPwm = myData.sliderValue.toInt();
  variable = myData.variable;
  
  if((variable== "stop")){
    stop_();
  }
  if((variable== "forward")){
    forward();
  }
  if((variable== "backward")){
    backward();
  }
  if((variable== "left")){
    left();
  }
  if((variable== "right")){
    right();
  }
  
 // || !strcmp(variable, "backward")|| !strcmp(variable, "forward") || !strcmp(variable, "left") || !strcmp(variable, "right")){
       
  
}

int get_left_right(){
  return digitalRead(LR_PIN);
}



/*
  Function Status
   CMtoSteps - Convert from centimeters to steps - DONE
   clearStop - Clears Emergency Stop condition - FUNCTIONAL
   getStatus - Returns current motor and controller status - NOT CODED
   motorAccel - Accelerates motor between two speeds - DONE
   motorDecel - Decelerates motor between two speeds - DONE
   motorMove - Moves motor at desired speed - DONE
   motorRPM - Reads encoder, returns speed in RPM - NOT CODED
   moveDistance - Move robot platform a specified distance in CM - NOT CODED
   movePWM - Moves motor at speed specified by PWM value - DONE
   moveRPM - Moves motor at speed specified by RPM value - NOT CODED
   readRPM - Reads motor speed in RPM - NOT CODED
   setI2C - Select I2C address based upon state of two I2C Address Pins - DONE
   setPWM - Select motor PWM Frequency based upon state of PWM FREQ Pin - DONE
   setPwmFrequency - Sets PWM frequency using a divisor - DONE
   stopMotor - Stops motor - DONE
*/


// Interrupt Service Routine
// Rotary Encoder input
// Interrupt 0

void isr_rotencode() {

}


// Interrupt Service Routine
// Emergency Stop input
// Interrupt 1

void isr_emstop() {
  // Set motor speed to zero
  motorPwm = 0;
  // Stop Motor by sending PWM signal low
  ledcWrite(MD10C_PWM_channel, 0);

  // Set Status
  motorStatus = 9;

}


// CMtoSteps Function
// Convert from centimeters to steps

int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = (wheel_diameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / count_rev;  // CM per Step

  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)

  return result;  // End and return result

}


// clearStop Function (I2C Command)
// Clears Emergency Stop condition

void clearStop()
{
  // Reset Motor Speed variable
  motorPwm = 0;

  // Write values to Motor Driver
  ledcWrite(MD10C_PWM_channel, motorPwm);

  // Set Status to Ready
  motorStatus = 1;
}


// getStatus Function (I2C Command)
// Returns current motor and controller status

void getStatus()
{

}


// motorAccel Function
// Accelerates motor between two speeds
// Required Low Speed and High Speed
// Optional Period of acceleration

void motorAccel(int mspeedlow, int mspeedhigh, int per = 1000) {

  // Determine the increment
  long incr;
  if (mspeedhigh > mspeedlow) {
    incr = per / (mspeedhigh - mspeedlow);
  } else {
    incr = 1;
  }

  for (int i = mspeedlow; i <= mspeedhigh; i++) {
    currentMillis = millis();
    while (millis() < currentMillis + incr) {
      // Drive motor for period in ms
      motorMove(i);
    }
  }
}


// motorDecel Function
// Decelerates motor between two speeds
// Required Low Speed and High Speed
// Optional Period of deceleration

void motorDecel(int mspeedlow, int mspeedhigh, int per = 1000) {

  // Determine the increment
  long incr;
  if (mspeedhigh > mspeedlow) {
    incr = per / (mspeedhigh - mspeedlow);
  } else {
    incr = 1;
  }

  for (int i = mspeedhigh; i >= mspeedlow; i--) {
    currentMillis = millis();
    while (millis() < currentMillis + incr) {
      // Drive motor for period in ms
      motorMove(i);
    }
  }
}


// motorMove Function
// Moves motor at desired speed
// Replaces analogWrite function by checking status

void motorMove(int mtrspeed) {
  // Check motor status
  if (motorStatus == 9) {
    ledcWrite(MD10C_PWM_channel, 0);
  } else {
    ledcWrite(MD10C_PWM_channel, mtrspeed);
  }

}


// motorRPM Function
// Reads encoder, returns speed in RPM

int motorRPM(int encpulse)
{

}


// moveDistance Function (I2C Command)
// Move robot platform a specified distance in CM
// Required Distance and Direction parameters
// Optional PWM Speed parameter - 0 to 255
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)

void moveDistance(int dis, int dr, int spp = 255, int acl = 0, int dcl = 0)
{

  // Set Status to Busy
  if (motorStatus != 9) {
    motorStatus = 0;
  }

  // Keep Direction between 0 and 1
  if (dr > 1) dr = 1;
  if (dr < 0) dr = 0;

  // Keep acceleration between 0 and 1 minute
  if (acl > 60000) acl = 60000;
  if (acl < 0) acl = 0;

  // Keep deceleration between 0 and 1 minute
  if (dcl > 60000) dcl = 60000;
  if (dcl < 0) dcl = 0;

  /**TODO**/


   if (motorStatus != 9) {
    motorStatus = 0;
  }
}


// movePWM Function (I2C Command)
// Moves motor at speed specified by PWM value
// Required Speed and Direction parameters
// Optional Run Time parameter in milliseconds - 1 ms to 60 seconds(0 = forever)
// Optional Acceleration parameter in milliseconds  - 1 ms to 60 seconds(0 = none)
// Optional Deceleration parameter in milliseconds  - 1 ms to 60 seconds(0 = none)

void movePWM(int spp, int dr, int tm = max_tm, int acl = min_dcl, int dcl = min_dcl)
{

  // Set Status to Busy
  if (motorStatus != 9) {
    motorStatus = 0;
  }

  // Keep PWM speed between 0 and 255
  spp = check_rng_speed(spp);
  
  // Keep Direction between 0 and 1
  dr = check_rng_dir(dr);
//  Serial.println("tm here");
//    Serial.println(tm);
  // Keep run time between 0 and 1 minute
  tm = check_rng_tm(tm);
  global_tm = tm;

  // Keep acceleration between 0 and 1 minute
   acl = check_rng_dcl(acl);

  // Keep deceleration between 0 and 1 minute
  dcl = check_rng_dcl(dcl);
  
  // Set Motor Direction
  digitalWrite(MD10C_DIR, dr);

  // Check if there is Acceleration specified
  // If Yes then call accelerate function
  if (acl > 0) {
    motorAccel(0, spp, acl);
    // Update the motor speed variable
    motorPwm = spp;
  }

  // Check if Run Time specified
  // If Yes run for desired time and check for deceletation
  // If no then run forever
  if (tm > 0) {
    Serial.println("tm > 0");
    Serial.println(tm);
    // Run for specified period
    currentMillis = millis();
    motorMove(spp);
    // Update the motor speed variable
    motorPwm = spp;
    while ((millis() < currentMillis + tm) && motorStatus==0 ) {
      delay(500);
//      Serial.println(motorStatus);
//      if (motorStatus != 0) {
//        Serial.println("in movePwm");
//        Serial.println(millis()- currentMillis);
//        Serial.println(motorStatus);
//        break;
//      }
      //wait for time in ms
      // check for new command.
      //motorMove(spp);
//      yield();
    }

    // Check for Deceleration
    if (dcl > 0) {
      // Call decelerate function
      motorDecel(0, spp, dcl);
      // Update the motor speed variable
      motorPwm = 0;
    } else {
      // Stop the motor
      ledcWrite(MD10C_PWM_channel, 0);
      // Update the motor speed variable
      motorPwm = 0;
    }

  } else {

    // Write speed to Motor Driver
    motorMove(spp);

  }

  // Set Status to Ready
  if (motorStatus != 9) {
    motorStatus = 1;
  }

}


// moveRPM Function (I2C Command)
// Moves motor at speed specified by RPM value
// Required Speed and Direction parameters
// Optional Run Time parameter - 1 ms to 60 seconds(0 = forever)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)
// Optional Acceleration parameter - 1 ms to 60 seconds(0 = none)

void moveRPM(int spr, int dr, int tm = 60000, int acl = 0, int dcl = 0)
{

  // Set Status to Busy
  //motorStatus = 0;

  // Keep Direction between 0 and 1
  if (dr > 1) dr = 1;
  if (dr < 0) dr = 0;

  // Keep run time between 0 and 1 minute
  if (tm > 60000) tm = 60000;
  if (tm < 0) tm = 0;

  // Keep acceleration between 0 and 1 minute
  if (acl > 60000) acl = 60000;
  if (acl < 0) acl = 0;

  // Keep deceleration between 0 and 1 minute
  if (dcl > 60000) dcl = 60000;
  if (dcl < 0) dcl = 0;



  // Set Status to Ready
  if (motorStatus != 9) {
   
    motorStatus = 1;
  }

}


// readRPM Function (I2C Command)
// Reads motor speed in RPM

void readRPM()
{

}



// stopMotor Function (I2C Command)
// Stops motor
// Optional Decelation parameter

void stopMotor(int dcl = min_dcl)
{

  // Set Status to Busy
  //motorStatus = 0;

  // Get current motor PWM speed
  int spp = motorPwm;

  // Keep deceleration between 0 and 1 minute
  dcl = check_rng_dcl(dcl);

  // Check for deceleration
  if (dcl > 0) {
    motorDecel(0, spp, dcl);
  }


  // Write values to motor driver to stop motor
  ledcWrite(MD10C_PWM_channel, 0);

  // Reset Motor Speed variable
  motorPwm = 0;


  // Set Status to Stop
  if (motorStatus != 9) {
   
    motorStatus = 8;
  }
}

TaskHandle_t Task1;

void forward(){
  digitalWrite(LED_STATUS,max_dir); //not very good style I know.
  digitalWrite(LED_LEFT, min_dir);
  digitalWrite(LED_EM_STOP,LOW);
//  xTaskCreatePinnedToCore(
//             Task1code,  /* Task function. */
//             "Task1",    /* name of task. */
//             10000,      /* Stack size of task */
//             NULL,       /* parameter of the task */
//             1,          /* priority of the task */
//             &Task1,     /* Task handle to keep track of created task */
//             1);         /* pin task to core 0 */
  movePWM(motorPwm, max_dir, 6000);
 //motorMove(motorPwm);
  
}

void backward(){
  
  digitalWrite(LED_STATUS,min_dir); //not very good style I know.
  digitalWrite(LED_LEFT, max_dir);
  digitalWrite(LED_EM_STOP,LOW);
  movePWM(motorPwm, min_dir, 0);
}

void left(){
  int glr = get_left_right();
  
  digitalWrite(LED_STATUS,glr); //not very good style I know.
  digitalWrite(LED_LEFT, !glr);
  digitalWrite(LED_EM_STOP,LOW);
  movePWM(motorPwm, glr, 0);
}

void right(){
   int glr = get_left_right();
  
  Serial.println(glr);
  digitalWrite(LED_STATUS,!glr); //not very good style I know.
  digitalWrite(LED_LEFT, glr);
  digitalWrite(LED_EM_STOP,LOW);
  movePWM(motorPwm, !glr, 0);
}

void stop_(){
  motorStatus=3;
  global_tm = min_tm;
  Serial.println(millis());
  Serial.println(motorStatus);
  stopMotor();
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_EM_STOP,HIGH);
   Serial.println("ESP now stop");
}





void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  
  for(;;){
    movePWM(motorPwm, max_dir, 6000);
    delay(200);
    vTaskDelete(NULL);
  }
}

void setup() {

  // Setup Serial Monitor for debugging                                                                                                                                     
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  
  Serial.println(" initializing ESP-NOW");
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP now intialised");
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  // Setup Pins as required

  pinMode(LR_PIN, INPUT);
  pinMode(LED_LEFT, OUTPUT);

  // Set Cytron Motor pins as Outputs
  pinMode(MD10C_PWM, OUTPUT);
  ledcSetup(MD10C_PWM_channel, freq, resolution);
  ledcAttachPin(MD10C_PWM, MD10C_PWM_channel);
  
 
  pinMode(MD10C_DIR, OUTPUT);

  // Set encoder as an Input with internal pullup
  //pinMode(ENC_IN, INPUT_PULLUP);

  // Set emergency stop as an Input with internal pullup
  pinMode(EM_STOP, INPUT_PULLUP);

  // Set Emergency Stop pins as Inputs with internal pullups
  pinMode(EM_STOP_0, INPUT_PULLUP);
  pinMode(EM_STOP_1, INPUT_PULLUP);
  pinMode(EM_STOP_2, INPUT_PULLUP);
  pinMode(EM_STOP_3, INPUT_PULLUP);
  pinMode(EM_STOP_4, INPUT_PULLUP);
  pinMode(EM_STOP_5, INPUT_PULLUP);

  

  // Set PWM Address Select pin as Input
//  pinMode(PWM_FREQ, INPUT);

  // Set Status and EM Stop LEDs as Outputs
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_EM_STOP, OUTPUT);

  // Set Processor Status as Output
 // pinMode(PROC_STATUS, OUTPUT);


  // Set the PWM Frequency
  //setPwmFrequency(MD10C_PWM, 8);
  //setPwmFrequency(MD10C_PWM, 1);

  //Determine I2C Slave Address
  //i2cAddr = setI2C(i2cAddr0,i2cAddr1,i2cAddr2,i2cAddr3);

  //Attach Emergency Stop Interrupt
  //attachInterrupt(EM_STOP, isr_emstop, FALLING);


  // Set Motor Status to Ready
  motorStatus = 1;


}


void loop() {

/*
  movePWM(150, 0, 10000, 5000, 3000);
  delay(2000);
  movePWM(250, 1, 8000, 5000, 3000);
  delay(2000);*/
  vTaskDelete(NULL);

}

// Matias Samora-Zappanti and Robert Christian, the Localization and Control side of Group 11. 
// Below is our code for all of the Mini-Project. The comments should help explain what things are.

// the wheels are named Cosmo and Wanda, to make it easy to keep track of them.
// Cosmo Wiring: Green to GND, Blue to VDD, Yellow to 2, White to 5
// Wanda Wiring: Green to GND, Blue to VDD, Yellow to 3, White to 6

#include <Wire.h> // Sending info to and from the raspberry pi
const int SLAVE_ADDRESS = 0x04;

const float pi = 3.1415926; // so I can call pi as a reference
volatile float COMPVISION_INPUTPOS; // the input from the computer vision people

double Cosmo_Desired_Position; // Cosmo's and Wanda's Desired Positions and some other smaller variables to go with those. 
double Cosmo_remainder;
volatile double Cosmo_Dist_Remain;
double Cosmo_Dist_To_Go_Net;

double Wanda_Desired_Position; 
double Wanda_remainder;
volatile double Wanda_Dist_Remain;
double Wanda_Dist_To_Go_Net;

const int Wheel_Enable_Pin = 4; // This is the enable pin for both wheels.

const int Cosmo_InterruptClk_Pin = 0; // interrupt pins (don't wire)
const int Wanda_InterruptClk_Pin = 1;

const int Cosmo_EncoderClk_Pin = 2; // encoder clk pins
const int Wanda_EncoderClk_Pin = 3;
const int Cosmo_EncoderDt_Pin = 5; // encoder dt pins
const int Wanda_EncoderDt_Pin = 6;

const int Cosmo_Direction_Pin = 7; // direction pins (don't wire)
const int Wanda_Direction_Pin = 8;
const int Cosmo_PWM_Pin = 9; // PWM pins (don't wire)
const int Wanda_PWM_Pin = 10;

volatile int Cosmo_ClkStatus; // ClkStatus, DtStatus, and LastClkStatus variables setup.
volatile int Cosmo_DtStatus;
volatile int Cosmo_LastClk_Status;
volatile int Wanda_ClkStatus;
volatile int Wanda_DtStatus;
volatile int Wanda_LastClk_Status;

volatile double Cosmo_Count = 0; // # of forward counts (for both)
volatile double Wanda_Count = 0; 

const float Battery_Voltage = 7.8; // hard set max voltage of the battery.

volatile double current_time_in_seconds; // values to keep track of time.
volatile double current_time;

volatile float Cosmo_desired_speed = 0.0; // speed variables to be edited later
volatile float Wanda_desired_speed = 0.0;

const float Kp = 3.100; // The Kp value
const float Ki = 1.200; //The Ki value
volatile float K_spin = 0.007; // This is a new variable I made up called K_spin, which scales the remaining distance to go (in counts) to a desired velocity (in rad/s). We tuned it to this value. 

volatile float Cosmo_error = 0.0000; // The errors and outputted voltages. We implemented a bit of an integral controller as well. 
volatile float Cosmo_Int = 0;
volatile float Cosmo_Voltage = 0.0000;
volatile double Wanda_error = 0.0000;
volatile float Wanda_Int = 0;
volatile float Wanda_Voltage = 0.0000;

unsigned int Cosmo_PWM = 0; // The variables to track the PWM output of the pins. 
unsigned int Wanda_PWM = 0;

const int desired_Ts_ms = 10; // The desired sample time, we have set to 10 milliseconds.
volatile double last_time_ms; // This is used for time math.

volatile float Wanda_rads_per_s = 0.0000; // Some variables for tracking speed/counts.
volatile float Cosmo_rads_per_s = 0.0000;
volatile double Wanda_LastSampleCount;
volatile double Cosmo_LastSampleCount;
volatile int time_since_last_sample;

const double counts_over_radians = (3200.0 / 6.283185); // Helps convert counts to radians.

volatile double x_pos; // The position variables, and their last-sample variables.
volatile double y_pos;
volatile double theta_pos;
volatile double x_old;
volatile double y_old;
volatile double theta_old;

const double b = 0.291; // 29.1 centimeters is 0.291 meters. b=distance between wheels.
const double wheel_radius = 0.0749; // the radius of the wheel is 7.49 centimeters, which is 0.0749 meters.

void setup() {
  pinMode(Cosmo_PWM_Pin, OUTPUT); // initializes directions and pwm pins as outputs, as well as the enable pin.
  pinMode(Wanda_PWM_Pin, OUTPUT);
  pinMode(Cosmo_Direction_Pin, OUTPUT);
  pinMode(Wanda_Direction_Pin, OUTPUT);
  pinMode(Wheel_Enable_Pin, OUTPUT);
  
  Wire.begin(SLAVE_ADDRESS); // sets up the transfer of data between the raspberry pi and the arduino
  Wire.onReceive(receiveData); 

  Serial.begin(115200); // sets up serial monitor
 
  Cosmo_ClkStatus = digitalRead(Cosmo_EncoderClk_Pin); // initialize status of Clk and Dt for Cosmo and Wanda, and sets their last states as well. 
  Cosmo_DtStatus = digitalRead(Cosmo_EncoderDt_Pin);
  Cosmo_LastClk_Status = Cosmo_ClkStatus;
  Wanda_ClkStatus = digitalRead(Wanda_EncoderClk_Pin);
  Wanda_DtStatus = digitalRead(Wanda_EncoderDt_Pin);
  Wanda_LastClk_Status = Wanda_ClkStatus;

  attachInterrupt(Cosmo_InterruptClk_Pin, Cosmo_ISR, CHANGE); // sets up the interrupts below. 
  attachInterrupt(Wanda_InterruptClk_Pin, Wanda_ISR, CHANGE);

  digitalWrite(Cosmo_Direction_Pin, LOW); //LOW = FORWARD. ( i know convention is that Low is backwards, but this has worked well for us so far)
  digitalWrite(Wanda_Direction_Pin, LOW); //LOW = FORWARD
  
  digitalWrite(Wheel_Enable_Pin, HIGH); //FIXME: MAKE SURE THIS IS CORRECT BEFORE RUNNING CODE ALWAYS

  last_time_ms = millis(); // sets the last_time_ms variable to be right now. 
  Wanda_LastSampleCount = 0; //initializes the counts to be zero.
  Cosmo_LastSampleCount = 0;

  x_pos = 0; // initializes all of the positions.
  y_pos = 0;
  theta_pos = 0;
  x_old = 0;
  y_old = 0;
  theta_old = 0;

}

void loop() {

  current_time = millis();

  Cosmo_Desired_Position = (COMPVISION_INPUTPOS * (pi / 2) * counts_over_radians);
  Wanda_Desired_Position = (COMPVISION_INPUTPOS * (-1) * (pi / 2) * counts_over_radians); // Desired positions converted to counts. Wanda's is negative since she's spinning the other direction. The inputted position is either 0, 1, 2, or 3. 

  Cosmo_remainder = (int(Cosmo_Desired_Position) % 2); // mod 2
  if (Cosmo_remainder > 1) { Cosmo_remainder = 2 - Cosmo_remainder; } 
  else { Cosmo_remainder = 0 - Cosmo_remainder; }
  Cosmo_Desired_Position = Cosmo_Desired_Position + Cosmo_remainder; // This makes it go to a set count value and stop, instead of jumping back and forth between two inachievable values. 
  
  Wanda_remainder = (int(Wanda_Desired_Position) % 2); // mod 2
  if (Wanda_remainder > 1) { Wanda_remainder = 2 - Wanda_remainder; } 
  else { Wanda_remainder = 0 - Wanda_remainder; }
  Wanda_Desired_Position = Wanda_Desired_Position + Wanda_remainder;

  
  Cosmo_Dist_Remain = Cosmo_Desired_Position - Cosmo_Count; // how far does he need to go, minus where he currently is. 
  Wanda_Dist_Remain = Wanda_Desired_Position - Wanda_Count; // how far does she need to go, minus where she currently is. 
  
  if (Cosmo_Dist_Remain > 1600) { Cosmo_Dist_Remain = Cosmo_Dist_Remain - 3200; } // This makes it go around in the proper direction; ie if it's 3/4 of a turn, it'll do a -1/4 turn instead. 
  if (Cosmo_Dist_Remain < -1600) { Cosmo_Dist_Remain = Cosmo_Dist_Remain + 3200; }
  
  if (Wanda_Dist_Remain > 1600) { Wanda_Dist_Remain = Wanda_Dist_Remain - 3200; }
  if (Wanda_Dist_Remain < -1600) { Wanda_Dist_Remain = Wanda_Dist_Remain + 3200; }
   
  Cosmo_desired_speed = (K_spin * Cosmo_Dist_Remain); //sets desired speed from the distance remaining. 
  Wanda_desired_speed = (K_spin * Wanda_Dist_Remain); 

  analogWrite(Cosmo_PWM_Pin, Cosmo_PWM); //writes the speed to the wheels
  analogWrite(Wanda_PWM_Pin, Wanda_PWM); 

  current_time = millis();

  if (current_time < 15000) { //for the first 15 seconds we print the time, distance remaining, and desired position for each wheel. 
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(Cosmo_Dist_Remain);
    Serial.print("\t");
    Serial.print(Wanda_Dist_Remain);
    Serial.print("\t");
    Serial.print(Cosmo_Desired_Position);
    Serial.print("\t");
    Serial.println(Wanda_Desired_Position);
  }

  
  if (current_time >= 2000) { // we want it to wait two second before starting the wheels, in case anything goes wrong. 
    
    Cosmo_error = Cosmo_desired_speed - Cosmo_rads_per_s; // sets that error code to be the voltage input. // i moved cosmo int to the sample thing so it doesn't go infinitely. 
    Cosmo_Voltage = (Kp*Cosmo_error) + Cosmo_Int; 
    if (Cosmo_Voltage>0) { // check the sign of voltage and set the motor driver sign pin as appropriate
      digitalWrite(Cosmo_Direction_Pin,LOW);
    } else {
      digitalWrite(Cosmo_Direction_Pin,HIGH);
    }
    Cosmo_PWM = (255*abs(Cosmo_Voltage))/Battery_Voltage;  // Apply the requested voltage, up to the maximum available
    analogWrite(Cosmo_PWM_Pin,min(Cosmo_PWM,255));

    Wanda_error = Wanda_desired_speed - Wanda_rads_per_s; // repeat for the other wheel aka Wanda.
    Wanda_Voltage = (Kp*Wanda_error) + Wanda_Int;
    if (Wanda_Voltage>0) {
      digitalWrite(Wanda_Direction_Pin,LOW);
    } else {
      digitalWrite(Wanda_Direction_Pin,HIGH);
    }
    Wanda_PWM = (255*abs(Wanda_Voltage))/Battery_Voltage;
    analogWrite(Wanda_PWM_Pin,min(Wanda_PWM,255));  

    if (abs(Cosmo_Voltage) >= Battery_Voltage) { // These check the voltages to make sure they aren't higher than the battery value, and if they are, it keeps their sign while re-assigning them the maximum they can be. 
      Cosmo_Voltage = ((Cosmo_Voltage * Battery_Voltage) / abs(Cosmo_Voltage));
    }
    if (abs(Wanda_Voltage) >= Battery_Voltage) {
      Wanda_Voltage = ((Wanda_Voltage * Battery_Voltage) / abs(Wanda_Voltage));
    }
  }
  
  current_time = millis(); // this again just in case the previous situation took a whole ms. 

  if ((current_time - last_time_ms) > desired_Ts_ms) { // if it's been a sample length;

    Cosmo_Int += Ki*Cosmo_error; //update the integral part of the integral control
    Wanda_Int += Ki*Wanda_error;
    
    time_since_last_sample = (current_time - last_time_ms); // insurance in case it goes over
    Wanda_rads_per_s = ((1000 * (Wanda_Count - Wanda_LastSampleCount) / (time_since_last_sample) ) / counts_over_radians); // outputs in rads per second.
    Cosmo_rads_per_s = ((1000 * (Cosmo_Count - Cosmo_LastSampleCount) / (time_since_last_sample) ) / counts_over_radians);
    
    current_time_in_seconds = current_time / 1000;

    x_pos = x_old + wheel_radius*((cos(theta_old)) * (((Wanda_Count - Wanda_LastSampleCount)+(Cosmo_Count - Cosmo_LastSampleCount))) / (counts_over_radians * 2));
    y_pos = y_old + wheel_radius*((sin(theta_old)) * (((Wanda_Count - Wanda_LastSampleCount)+(Cosmo_Count - Cosmo_LastSampleCount))) / (counts_over_radians * 2));
    theta_pos = theta_old + ( ( wheel_radius * ( (Cosmo_Count - Cosmo_LastSampleCount) - (Wanda_Count - Wanda_LastSampleCount) ) ) / ( counts_over_radians * b ) ); // b is the distance between the wheels
    
    x_old = x_pos;
    y_old = y_pos;
    theta_old = theta_pos; 

    Wanda_LastSampleCount = Wanda_Count; // update for next loop
    Cosmo_LastSampleCount = Cosmo_Count;
    last_time_ms = millis(); 
    
  }
  
}

void receiveData(int byteCount) {
  while (Wire.available()) {
    COMPVISION_INPUTPOS = Wire.read(); // receive byte as an integer, in the form 0, 1, 2, or 3, to correspond to 0, pi/2, pi, 3pi/2. 
  }
}

// Below this point are the ISR's for both wheel encoders. This is basically copied from Assignment 1. 

void Cosmo_ISR() { // if the CLK value changes,
  Cosmo_ClkStatus = digitalRead(Cosmo_EncoderClk_Pin); // read in both pins.
  Cosmo_DtStatus = digitalRead(Cosmo_EncoderDt_Pin);
  if (Cosmo_ClkStatus != Cosmo_LastClk_Status) { // (sometimes the ISR triggers on accident. this stops that.)
    Cosmo_LastClk_Status = Cosmo_ClkStatus; // sets the value for the last status, for next time. 
    if (Cosmo_DtStatus == Cosmo_ClkStatus) { Cosmo_Count = Cosmo_Count + 2; } // if CLK changed which thus made clk and dt the same, that means the encoder turned FORWARD FOR COSMO. 
    else { Cosmo_Count = Cosmo_Count - 2; } // otherwise, that means the encoder turned BACKWARD FOR COSMO.
    while (Cosmo_Count >= 3200) { Cosmo_Count = Cosmo_Count - 3200; }
    while (Cosmo_Count <= -3200) { Cosmo_Count = Cosmo_Count + 3200; }
  } 
}
void Wanda_ISR() { // if the CLK value changes,
  Wanda_ClkStatus = digitalRead(Wanda_EncoderClk_Pin); // read in both pins.
  Wanda_DtStatus = digitalRead(Wanda_EncoderDt_Pin);
  if (Wanda_ClkStatus != Wanda_LastClk_Status) { // (sometimes the ISR triggers on accident. this stops that.)
    Wanda_LastClk_Status = Wanda_ClkStatus; // sets the value for the last status, for next time. 
    if (Wanda_DtStatus == Wanda_ClkStatus) { Wanda_Count = Wanda_Count - 2; }  // if CLK changed which thus made clk and dt the same, that means the encoder turned BACKWARD FOR WANDA.
    else { Wanda_Count = Wanda_Count + 2; } // otherwise, that means the encoder turned FORWARD FOR WANDA.
    while (Wanda_Count >= 3200) { Wanda_Count = Wanda_Count - 3200; }
    while (Wanda_Count <= -3200) { Wanda_Count = Wanda_Count + 3200; }
  } 
}

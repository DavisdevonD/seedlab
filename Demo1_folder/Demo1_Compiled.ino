// Robert Christian and Matias Samora-Zappanti
// Demo 1 Code
// Compiled from previous projects and individual Arduino code files. 



//OVERARCHING VARIABLES:
const float pi = 3.1415926;
const double b = 0.291; // Distance between the wheels.
const double wheel_radius = 0.0749; // Radius of each wheel. 
const double counts_over_radians = (3200.0 / 6.283185); // Helps convert counts to radians.
const int desired_Ts_ms = 10; // The desired sample time, we have set to 10 milliseconds.
const float Battery_Voltage = 7.8; // Max voltage of the battery.

const int Cosmo_InterruptClk_Pin = 0; // These are the pins associated with each wheel.
const int Wanda_InterruptClk_Pin = 1;
const int Cosmo_EncoderClk_Pin = 2;
const int Wanda_EncoderClk_Pin = 3;
const int Wheel_Enable_Pin = 4;
const int Cosmo_EncoderDt_Pin = 5;
const int Wanda_EncoderDt_Pin = 6;
const int Cosmo_Direction_Pin = 7;
const int Wanda_Direction_Pin = 8;
const int Cosmo_PWM_Pin = 9;
const int Wanda_PWM_Pin = 10;
const int LED_PIN = 13;

volatile int Cosmo_ClkStatus; // ClkStatus, DtStatus, and LastClkStatus variables setup.
volatile int Cosmo_DtStatus;
volatile int Cosmo_LastClk_Status;
volatile int Wanda_ClkStatus;
volatile int Wanda_DtStatus;
volatile int Wanda_LastClk_Status;

volatile double Cosmo_Count = 0; // # of forward counts for both wheels. 
volatile double Wanda_Count = 0; 

volatile float Wanda_rads_per_s = 0.0000; // Variables for tracking speed/counts.
volatile float Cosmo_rads_per_s = 0.0000;

volatile double Cosmo_LastSampleCount; // Tracks how many counts have passed since last sample. 
volatile double Wanda_LastSampleCount;

volatile float Cosmo_desired_speed = 0.0; // Desired speed variables for each wheel. 
volatile float Wanda_desired_speed = 0.0;

volatile float Cosmo_Voltage = 0.0000; // The voltage variables to be sent to the PWM's. 
volatile float Wanda_Voltage = 0.0000;

unsigned int Cosmo_PWM = 0; // The variables to track the PWM output of the pins. 
unsigned int Wanda_PWM = 0;

volatile double current_time = 0.000; // These three are used for time math. 
volatile double last_time_ms;
volatile int time_since_last_sample;

volatile double x_pos; // The position variables, and their last-sample versions.
volatile double y_pos;
volatile double theta_pos_current;
volatile double x_old;
volatile double y_old;
volatile double theta_pos_old;



// SPIN VARIABLES:
volatile double THETA_DESIRED_TA; // Inputted desired theta, given from the TA. (In radians)
volatile double theta_error; // Difference between current angle and desired angle. 

double theta_speed_desired; // Speed variables for turning. 
double theta_speed_current; 
double theta_speed_error;

const float K_theta_spin = 4.000; // Higher value makes it spin faster.
const float Ki_Kp_spin_ratio = 0.400 ; // Amount of Ki vs. Kp. 0 is absolute proportional, 1 is absolute integral.
float Kp_spin = (1.000 - Ki_Kp_spin_ratio); // Too much Kp makes him not strong enough.
float Ki_spin = (Ki_Kp_spin_ratio); // Too much Ki can make him slip and shake. 

volatile double Cosmo_spin_error = 0.000; // The errors and integral portions of the spin controller.
volatile double Cosmo_Spin_Int = 0.000;
volatile double Wanda_spin_error = 0.000;
volatile double Wanda_Spin_Int = 0.000;



// DRIVE VARIABLES:
volatile double DISTANCE_DESIRED_TA_FT; // The inputted desired distance, given from the TA. (In feet)
volatile double dist_to_go; // Variables for figuring out how far he has to go. 
volatile double x_desired;
volatile double y_desired;
volatile double x_to_go;
volatile double y_to_go;

volatile double x_start; // The start points for any run.
volatile double y_start;
volatile double dist_gone; // Distance gone so far. 

volatile double pos_speed_desired = 0.0000; // Some speed variables for driving. 
volatile double pos_speed_error = 0.0000;
volatile double pos_speed_current = 0.0000;

volatile float K_pos = 5.000; // See above explanations of K values for spin; these are the same but for drive. 
volatile float Ki_Kp_Pos_Ratio = 0.200;
volatile float Kp_pos = (1.000 - Ki_Kp_Pos_Ratio); 
volatile float Ki_pos = Ki_Kp_Pos_Ratio;
volatile double Kd; // The entirety of the derivative controller. 

volatile double Cosmo_pos_speed_error = 0.0000; // The errors and integral portions of the drive controller.
volatile double Cosmo_pos_Int = 0.0000;
volatile double Wanda_pos_speed_error = 0.0000;
volatile double Wanda_pos_Int = 0.0000;



// FREEZE VARIABLES:
volatile double Cosmo_freeze_speed_error = 0.0000; // The errors and integral portions of the freeze controller.
volatile double Cosmo_freeze_Int = 0.0000;
volatile double Wanda_freeze_speed_error = 0.0000;
volatile double Wanda_freeze_Int = 0.0000;

volatile double Wanda_freeze; // Positions for where the wheels should freeze. 
volatile double Cosmo_freeze;

volatile float Kp_freeze = 1.000; // The Ki and Kp for the freeze controller. 
volatile float Ki_freeze = 1.000;



// STATE VARIABLES:
const int Idle_Start =  1; // These are the state variables for what Timmy should be doing. 
const int Spin =        2;
const int Drive =       3;
const int Idle_Freeze = 4;
const int Spin_Setup =  5;
const int Drive_Setup = 6;
volatile int Timmy_Status; 



// OVERSHOOT VARIABLES
volatile float spin_percent_extra; // These variables exist to compensate for slipping. 
volatile float distance_percent_extra;



void setup() {
  // INPUTS:
  THETA_DESIRED_TA = 1.000*pi;     // TA given distance to turn (in radians)
  DISTANCE_DESIRED_TA_FT = 1.000;                // TA given distance to drive (in feet)


  
  // OVERARCHING SETUP:
  Timmy_Status = Idle_Start; // Timmy starts in the Idle_Start phase. 
  
  pinMode(Cosmo_PWM_Pin, OUTPUT); // Initializes directions and pwm pins as outputs, as well as the enable pin.
  pinMode(Wanda_PWM_Pin, OUTPUT);
  pinMode(Cosmo_Direction_Pin, OUTPUT);
  pinMode(Wanda_Direction_Pin, OUTPUT);
  pinMode(Wheel_Enable_Pin, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH); // Turns the LED on while we are setting up. 

  digitalWrite(Cosmo_Direction_Pin, LOW); //LOW = FORWARD 
  digitalWrite(Wanda_Direction_Pin, LOW); //LOW = FORWARD
  digitalWrite(Wheel_Enable_Pin, HIGH); // HIGH = ON
  
  Cosmo_ClkStatus = digitalRead(Cosmo_EncoderClk_Pin); // Initializes status of Clk and Dt for Cosmo and Wanda and their last states as well. 
  Cosmo_DtStatus = digitalRead(Cosmo_EncoderDt_Pin);
  Cosmo_LastClk_Status = Cosmo_ClkStatus;
  Wanda_ClkStatus = digitalRead(Wanda_EncoderClk_Pin);
  Wanda_DtStatus = digitalRead(Wanda_EncoderDt_Pin);
  Wanda_LastClk_Status = Wanda_ClkStatus;

  Serial.begin(115200); // Starts monitor.
  attachInterrupt(Cosmo_InterruptClk_Pin, Cosmo_ISR, CHANGE); // Sets up the interrupts below. 
  attachInterrupt(Wanda_InterruptClk_Pin, Wanda_ISR, CHANGE);

  last_time_ms = millis(); // Sets the last_time_ms variable to be right now. 
  Wanda_LastSampleCount = 0; // Initializes the counts to be zero.
  Cosmo_LastSampleCount = 0;
  
  x_pos = 0.000; // Initializes all of the positions.
  y_pos = 0.000;
  theta_pos_current = 0.000;
  x_old = 0.000;
  y_old = 0.000;
  theta_pos_old = 0.000;

}

void loop() {

  current_time = millis();

  switch(Timmy_Status) {
    case Idle_Start:
      digitalWrite(LED_PIN, HIGH); // Turns the LED on in this phase.
      digitalWrite(Wheel_Enable_Pin, LOW); 
      delay(2000);
      digitalWrite(Wheel_Enable_Pin, HIGH); // Lets the wheels spin for two seconds. 
      Timmy_Status = Spin_Setup; // Next phase is Spin_Setup. 
      break;



    case Spin:
      digitalWrite(LED_PIN, LOW); // Turns the LED off for this phase. 
      theta_error = THETA_DESIRED_TA - theta_pos_current; // Get error between desired and current theta. 
      theta_speed_desired = K_theta_spin*(theta_error / abs(theta_error)) * min(0.5,abs(theta_error)); // Set desired speed proportional to that, at a max of Kp*0.5
      theta_speed_error = theta_speed_desired - theta_speed_current; // Get error between desired speed and current speed. 

      if ( (abs(theta_error) < 0.005) && (abs(theta_speed_current) < 0.001) && (abs(Wanda_rads_per_s) < 0.01) && (abs(Cosmo_rads_per_s) < 0.01) ) {
        theta_error = 0;
        theta_speed_error = 0;
        Timmy_Status = Drive_Setup;
      } // If we're done, move on to the next phase which is Drive_Setup. 

      Cosmo_desired_speed = (theta_speed_error); // Set the wheel speeds to be the desired speeds (in opposite directions.)
      Wanda_desired_speed = -1 * (theta_speed_error);

      Cosmo_spin_error = Cosmo_desired_speed - Cosmo_rads_per_s; // Get the error between the desired and current speed for Cosmo and Wanda (the wheels)
      Wanda_spin_error = Wanda_desired_speed - Wanda_rads_per_s; 

      Cosmo_Voltage = (Kp_spin*Cosmo_spin_error) + Cosmo_Spin_Int; // Sets the voltage with PI (the I is set later)
      Wanda_Voltage = (Kp_spin*Wanda_spin_error) + Wanda_Spin_Int;

      Serial.print(theta_pos_current); // Some print statements from when we were troubleshooting. 
      Serial.print("\t");
      Serial.print(K_theta_spin);
      Serial.print("\t");
      Serial.print(Kp_spin);
      Serial.print("\t");
      Serial.print(Ki_spin);
      Serial.print("\t");
      Serial.print("SPIN");
      Serial.print("\t");
      Serial.println();
      break;



    case Drive:
      digitalWrite(LED_PIN, LOW); // Turns off the LED for this phase. 
      dist_gone = sqrt( ((x_pos - x_start)*(x_pos - x_start)) + ((y_pos - y_start)*(y_pos - y_start)) ); // Figures out how much he has gone.  
      pos_speed_desired = K_pos * (dist_to_go - dist_gone) - Kd; // Sets desired speed based on that. 
      pos_speed_desired = min(pos_speed_desired, 5); // Sets a maximum speed at 5.  
      pos_speed_error = pos_speed_desired - pos_speed_current; // Gets the error between the desired and the current speeds.

      if ( ( (dist_to_go - dist_gone) < 0.005 ) && ( abs(pos_speed_current) < 0.005 ) && ( abs(Wanda_rads_per_s) < 0.01 ) && ( abs(Cosmo_rads_per_s) < 0.01 ) ) {
        pos_speed_error = 0;
        Cosmo_freeze = Cosmo_Count;
        Wanda_freeze = Wanda_Count;
        Timmy_Status = Idle_Freeze;
      } // If we're done, move on to the next phase which is Idle_Freeze. 

      Cosmo_desired_speed = pos_speed_desired * (1.0170); // Sets the speed to the wheels. Cosmo runs a little slower, so we needed to boost him. 
      Wanda_desired_speed = pos_speed_desired;

      if ((Cosmo_rads_per_s - Wanda_rads_per_s) > 0) { Wanda_desired_speed = Wanda_desired_speed + (Cosmo_rads_per_s - Wanda_rads_per_s); } // If one wheel is faster, add the difference to the other wheel. 
      else if ((Wanda_rads_per_s - Cosmo_rads_per_s) > 0) { Cosmo_desired_speed = Cosmo_desired_speed + (Wanda_rads_per_s - Cosmo_rads_per_s); } 

      Cosmo_pos_speed_error = (Cosmo_desired_speed - Cosmo_rads_per_s); // Gets the difference between the desired and current speeds for the wheels. 
      Wanda_pos_speed_error = (Wanda_desired_speed - Wanda_rads_per_s);

      Cosmo_Voltage = (Kp_pos*Cosmo_pos_speed_error) + Cosmo_pos_Int; // Sets the voltage with PI (the I is set later) 
      Wanda_Voltage = (Kp_pos*Wanda_pos_speed_error) + Wanda_pos_Int;

      Serial.print(dist_gone); // Some print statements from when we were troubleshooting. 
      Serial.print("\t");
      Serial.print("DRIVE");
      Serial.print("\t");
      Serial.println();
      break;



    case Idle_Freeze:
      digitalWrite(LED_PIN, HIGH); // Turns the LED back on. 
      Cosmo_desired_speed = (Cosmo_freeze - Cosmo_Count); // The desired speed should be 0, but if the wheels have spun, spin them back. 
      Wanda_desired_speed = (Wanda_freeze - Wanda_Count);

      Cosmo_freeze_speed_error = 0 - Cosmo_rads_per_s; // Gets the error between the desired speed (0) and the current speed.
      Wanda_freeze_speed_error = 0 - Wanda_rads_per_s;

      Cosmo_Voltage = (Kp_freeze*Cosmo_freeze_speed_error) + Cosmo_freeze_Int; // Sets the voltage with PI (the I is set later)
      Wanda_Voltage = (Kp_freeze*Wanda_freeze_speed_error) + Wanda_freeze_Int;

      Timmy_Status = Idle_Freeze;
      break; 



    case Spin_Setup:
      digitalWrite(LED_PIN, HIGH); // Turns the LED on while in this phase. 
      THETA_DESIRED_TA = THETA_DESIRED_TA; // (Placeholder line of code for when this will be changed by the RPi.  
      spin_percent_extra = 1.000 + (0.0150 * ( (pi/2.000) / (abs(THETA_DESIRED_TA) + 0.240) ) );
      THETA_DESIRED_TA = THETA_DESIRED_TA * spin_percent_extra; // This is the aforementioned compensation for slipping. 

      Cosmo_Spin_Int = 0.000; // Reinitializes the Integral portions of the controller to 0. 
      Wanda_Spin_Int = 0.000;
      Timmy_Status = Spin; // Next phase is Spin. 
      break;



    case Drive_Setup:
      digitalWrite(LED_PIN, HIGH); // Turns the LED on while in this phase. (It'll likely barely blink.)
      dist_to_go = ( DISTANCE_DESIRED_TA_FT / (3.2808399) ); // The given distance to go, but converted to meters. 
      distance_percent_extra = ( 94.61906 / (abs(dist_to_go) + 95) );
      dist_to_go = dist_to_go * (distance_percent_extra); // Overshoot value to account for slipping. 
      
      dist_gone = 0.000; // We haven't moved yet, so dist_gone should reinitialize to zero every run. 
      x_desired = dist_to_go * cos(theta_pos_current); // The desired x and y point for the angle we should be going to. 
      y_desired = dist_to_go * sin(theta_pos_current);

      x_start = x_pos; // Start position is current position before driving. (This will prolly be zero.)
      y_start = y_pos;

      x_to_go = x_desired - x_pos; // Distance to go, accounting for the distance already moved.
      y_to_go = y_desired - y_pos;

      dist_to_go = sqrt( (x_to_go * x_to_go) + (y_to_go * y_to_go) ); // Distance to go is straight-line distance. 
      Kd = K_pos * (dist_to_go - dist_gone); // Helps correct slipping at the beginning. 

      Cosmo_pos_Int = 0.000; // Reinitializes Integral portions of the controller to 0. 
      Wanda_pos_Int = 0.000;
      Timmy_Status = Drive; // The next phase is Drive. 
      break;
  }



  if (Cosmo_Voltage>0) { digitalWrite(Cosmo_Direction_Pin,LOW); } else { digitalWrite(Cosmo_Direction_Pin,HIGH); } // Set the directions of the wheels. 
  if (Wanda_Voltage>0) { digitalWrite(Wanda_Direction_Pin,LOW); } else { digitalWrite(Wanda_Direction_Pin,HIGH); }

  Cosmo_PWM = (255*abs(Cosmo_Voltage))/Battery_Voltage;  // Set the requested voltage to the PWM. 
  Wanda_PWM = (255*abs(Wanda_Voltage))/Battery_Voltage;
  
  analogWrite(Cosmo_PWM_Pin, min(Cosmo_PWM, 255)); // Write the voltage to the wheels, with a max of 255 PWM. 
  analogWrite(Wanda_PWM_Pin, min(Wanda_PWM, 255));

  current_time = millis();
  if ((current_time - last_time_ms) > desired_Ts_ms) { // Sample every sample length. 
    time_since_last_sample = (current_time - last_time_ms);

    Wanda_rads_per_s = ((1000 * (Wanda_Count - Wanda_LastSampleCount) / (time_since_last_sample) ) / counts_over_radians); // Gets current speeds. 
    Cosmo_rads_per_s = ((1000 * (Cosmo_Count - Cosmo_LastSampleCount) / (time_since_last_sample) ) / counts_over_radians); 

    x_pos = x_old + wheel_radius*((cos(theta_pos_old)) * (((Wanda_Count - Wanda_LastSampleCount)+(Cosmo_Count - Cosmo_LastSampleCount))) / (counts_over_radians * 2)); // Gets current x, y, and theta
    y_pos = y_old + wheel_radius*((sin(theta_pos_old)) * (((Wanda_Count - Wanda_LastSampleCount)+(Cosmo_Count - Cosmo_LastSampleCount))) / (counts_over_radians * 2));
    theta_pos_current = theta_pos_old + ( ( wheel_radius * ( (Cosmo_Count - Cosmo_LastSampleCount) - (Wanda_Count - Wanda_LastSampleCount) ) ) / ( counts_over_radians * b ) ); 

    x_old = x_pos; // Resets "old" variables.
    y_old = y_pos;
    theta_pos_old = theta_pos_current; 
    Wanda_LastSampleCount = Wanda_Count;
    Cosmo_LastSampleCount = Cosmo_Count;
    last_time_ms = millis();
    
    if ( Timmy_Status == Spin ) { // SPIN-SPECIFIC SAMPLE CODE:
      theta_speed_current = (wheel_radius * ( Cosmo_rads_per_s - Wanda_rads_per_s ) ) / b; // Gets current spin speed.
      Wanda_Spin_Int += Ki_spin*Wanda_spin_error; // Updates the integral portions of the spin speed. 
      Cosmo_Spin_Int += Ki_spin*Cosmo_spin_error;
    }

    if ( Timmy_Status == Drive ) { // DRIVE-SPECIFIC SAMPLE CODE:
      pos_speed_current = (wheel_radius * (Cosmo_rads_per_s + Wanda_rads_per_s)) / 2; // Gets current drive speed. 
      Cosmo_pos_Int += Ki_pos*Cosmo_pos_speed_error; // Updates the integral portions of the drive speed. 
      Wanda_pos_Int += Ki_pos*Wanda_pos_speed_error;
      Kd = 0.991 * Kd; // Updated the "D" portion of the drive speed. 
    }

    if ( Timmy_Status == Idle_Freeze) { // FREEZE-SPECIFIC SAMPLE CODE
      Cosmo_freeze_Int += Ki_freeze * Cosmo_freeze_speed_error; // Updates the integral portions of the freeze speed. 
      Wanda_freeze_Int += Ki_freeze * Wanda_freeze_speed_error;
    }
  }
}



void Cosmo_ISR() { // if the CLK value changes,
  Cosmo_ClkStatus = digitalRead(Cosmo_EncoderClk_Pin); // Read in both pins. 
  Cosmo_DtStatus = digitalRead(Cosmo_EncoderDt_Pin);
  if (Cosmo_ClkStatus != Cosmo_LastClk_Status) { // (Sometimes the ISR triggers on accident. This stops that.)
    Cosmo_LastClk_Status = Cosmo_ClkStatus; // Sets the value for the last status, for next time. 
    if (Cosmo_DtStatus == Cosmo_ClkStatus) { Cosmo_Count = Cosmo_Count + 2; } // If CLK changed which thus made Clk and Dt the same, that means the encoder turned FORWARD FOR COSMO. 
    else { Cosmo_Count = Cosmo_Count - 2; } // Otherwise, that means the encoder turned BACKWARD FOR COSMO.
  } 
}
void Wanda_ISR() { // See above comments. 
  Wanda_ClkStatus = digitalRead(Wanda_EncoderClk_Pin); 
  Wanda_DtStatus = digitalRead(Wanda_EncoderDt_Pin);
  if (Wanda_ClkStatus != Wanda_LastClk_Status) { 
    Wanda_LastClk_Status = Wanda_ClkStatus; 
    if (Wanda_DtStatus == Wanda_ClkStatus) { Wanda_Count = Wanda_Count - 2; }  // If CLK changed which thus made Clk and Dt the same, that means the encoder turned BACKWARD FOR WANDA. 
    else { Wanda_Count = Wanda_Count + 2; } // Otherwise, that means the encoder turned FORWARD FOR WANDA.
  } 
}

#include "TimerStepper.h"
#include "board_list.h"

// --------------------------------------- SERIAL PARAMETERS -------------------------------------------------------- //

String InStr;          // Variable String to receive incoming Serial commands
char first_char;       // First Character of the incoming String. Contains command code (which command to execute)
//int myArray[i+1] = {0, ... , i};
// Array example --> int myArray[3] = {a0, a1, a2} ; myArray[2] contains a2
float sub_command[9];  // Array to store the parameters passed with the command (angle, velocity, acceleration, etc...). Need to specify maximum number of expected parameters
int sub_command_size = sizeof(sub_command) / sizeof(sub_command[0]);  // sizeof operator returns the size of the object in bytes
int params_received = 0;
bool SerialOut = false;
int sample_time = 100; // Main loop cycle time
int motor_id;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  TimerStepper::InitialiseTimers();

  timerSteppers[0].MotorSetup(22, 23, 24, 200, true, 5);

  // Sample Commands:
  // M:0;200;0;500;20;20;
  // R:0;5;2;10;10;
  // L:

  //timerSteppers[1].MotorSetup(5, 6, 7, 200, true, 2);

}

void loop() {
  uint32_t start = millis(); // read start time
  // 1 - READ INCOMING SERIAL COMMAND
  readCommand(InStr);

  if(SerialOut){
    Serial.print(first_char);Serial.print("\t");
    Serial.print(params_received);
    Serial.println("\t");
  }

  uint32_t finish = millis();  // read finish time
  //Calculate cycle time duration
  int duration = (int) finish - (int) start;
  //Serial.println(duration);
  /*To obtain a fixed sample time, the cycle time duration is subtracted   
  from the sample time, and the difference is added to the code as a delay*/
  int t_delay = sample_time - duration;
  if (t_delay > 0) {delay(t_delay);}  

}


// SERIAL COMMUNICATIONS -------------------------------------------------------

void readCommand(String &inString) {
  //If Serial available read the input Serial line
  if (Serial.available()) {
    inString = Serial.readStringUntil('\n');

    //If the string is not empty, translate and execute the command
    if (inString.length() > 0) {
      //SplitString extracts the command code and stores it into first_char (what command to execute), then extracts the command parameters
      //and stores them into the array sub_command (information about the command)
      splitString(inString, first_char, sub_command);
      for (int i = 0; i <= 8; i++) {if(sub_command[i] != -999){params_received++;}}
      switch (first_char) {
        case 'I':  //I:on_off;
          if(sub_command[0]!=0 && sub_command[0]!=1){
            first_char = 'E';
            break;
          }
          if((bool)sub_command[0]){
            SerialOut = true; 
              for (int i = 0; i <= 8; i++){
                timerSteppers[i].StopMotor();
                timerSteppers[i].ResetPosition();
              }
          }else{
            SerialOut = false; 
          }

        break;
        case 'C':  //CONFIGURE MOTOR C:motor_id;pls_pin;dir_pin;ena_pin;h_sw_pin;f_sw_pin;ppr;pitch;travel_mm;
          //for (int i = 0; i <= 8; i++) {if(sub_command[i] == -999){return;}}
          //for (int i = 0; i <= 8; i++) {if(sub_command[i] != -999){params_received++;}}
          motor_id = (int)sub_command[0];
          if (motor_id < 0 || motor_id > 8){
            first_char = 'E';
            break;
          }

          if (params_received == 1){
            Serial.println("C;" + timerSteppers[motor_id].PassMotorParametersAndIdToString());
          }else if (params_received == 9){
            timerSteppers[motor_id].MotorSetup((int)sub_command[1], (int)sub_command[2], (int)sub_command[3], (int)sub_command[6], true, sub_command[7]);
            //timerSteppers[motor_id].LimitSwtichesSetup((int)sub_command[4], (int)sub_command[5], 0, sub_command[8]);
          }else{
            first_char = 'E';
            //return;            
          }
        break;

        case 'H':  //H:motor_id;mode;param1;param2;
          // Get Motor id   
          motor_id = (int)sub_command[0];
          if (motor_id < 0 || motor_id > 8){
            first_char = 'E';
            break;
          }
          // Check Home mode (normal / override)
          // If normal home mode
          if (sub_command[1] == 0 && params_received == 4){
            timerSteppers[motor_id].Home((int)sub_command[2], (bool)sub_command[3]);            
          }else if (sub_command[1] == 1 && params_received == 3){
            timerSteppers[motor_id].Home_ovr(sub_command[2]);
          }else{
            first_char = 'E';
          }
        break;

        case 'M':  //MOVE STEPS M:motor_id;steps;dir;freq;acc_steps;dec_steps;
          if(params_received == 6){
            motor_id = (int)sub_command[0];
            if (motor_id < 0 || motor_id > 8){
              first_char = 'E';
              break;
            }
            // Serial.println("Command Received!");
            timerSteppers[motor_id].RotateSteps((int)sub_command[1], (bool)sub_command[2], (int)sub_command[3], (int)sub_command[4], (int)sub_command[5]);
          }else{
            first_char = 'E';
          }          
        break;

        case 'R':  //RELATIVE LINEAR MOTION R:motor_id;dist_mm;vel_mms;acc_mms;dec_mms;
          if(params_received == 5){
            motor_id = (int)sub_command[0];
            if (motor_id < 0 || motor_id > 8){
              first_char = 'E';
              break;
            }
            timerSteppers[motor_id].RunLinearStage(sub_command[1], sub_command[2], sub_command[3], sub_command[4]);
          }else{
            first_char = 'E';
          }
        break;

        case 'L':  //ABSOLUTE LINEAR MOTION R:motor_id;abs_trgPos_mm;vel_mms;acc_mms;dec_mms;
          if(params_received == 5){
            motor_id = (int)sub_command[0];
            if (motor_id < 0 || motor_id > 8){
              first_char = 'E';
              break;
            }
            timerSteppers[motor_id].RunLinearStage_abs(sub_command[1], sub_command[2], sub_command[3], sub_command[4]);
          }else{
            first_char = 'E';
          }
        break;

        case 'V':  //SPEED MODE V:motor_id;dir;vel_mms;acc_mms;
          if(params_received == 4){
            motor_id = (int)sub_command[0];
            if (motor_id < 0 || motor_id > 8){
              first_char = 'E';
              break;
            }
          }else{
            first_char = 'E';
          }
        break;

        // TODO: need to create statid public int StepperTimer::count to get the number of stepper motors. Then, loop from 0 to count and stop each motor

        // case 'S':  //STOP MOTION S:motor_id;stop_mode;param1;param2;
        //   for (int i = 0; i <= 1; i++) {if(sub_command[i] == -999){return;}}
        //   for (int i = 2; i <= 3; i++) {if(sub_command[i] != -999){params_received++;}}
        //   motor_id = (int)sub_command[0];
        //   if(motor_id == -1){ // Emergency Stop - Stop All motors immediately
        //     timerSteppers[0].StopMotor();
        //     timerSteppers[1].StopMotor();
        //     timerSteppers[2].StopMotor();
        //     timerSteppers[3].StopMotor();
        //     timerSteppers[4].StopMotor();
        //     timerSteppers[5].StopMotor();
        //     timerSteppers[6].StopMotor();
        //     timerSteppers[7].StopMotor();
        //     timerSteppers[8].StopMotor();
        //   }        
        //   if (motor_id < 0 || motor_id > 8){return;}
        //   if ((int)sub_command[1] == 0 && params_received == 0){timerSteppers[motor_id].StopMotor();}
        //   else if ((int)sub_command[1] == 1 && params_received == 1){timerSteppers[motor_id].StopSpeedMode((int)sub_command[2]);}
        //   else if ((int)sub_command[1] == 2 && params_received == 2){timerSteppers[motor_id].StopWithDeceleration(sub_command[2], sub_command[3]);}
        // break;  
   

      }
    }

    // Reset variables
    inString = "";
    //sub_command_size
    int array_size = sizeof(sub_command) / sizeof(sub_command[0]);  // sizeof operator returns the size of the object in bytes
    for (int i = 0; i <= (sub_command_size - 1); i++) {
    //for (int i = 0; i <= (array_size - 1); i++) {
      sub_command[i] = -999;  // Initialise array with -999. Check inside each case, that the split values are not equal to -999.
    } 
    params_received = 0;
  }
}

void splitString(String &inString, char &fst_char, float c[]) {
  //Function to split the entering command into its respective values.
  //The command should have a first character indicating the action code (home, move, ...) followed by a serie of parameters (position, velocity, time,...)
  // The action code is followed by ":", and every parameter is separated by ";"
  // The function takes the inStr command and updates by reference the firstChar parameter and the sub_command array of values

  fst_char = inString.charAt(0);
  int ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9, ind10;
  ind1 = inString.indexOf(':');  //Find the index of the character ':'
  if (ind1 == -1) { //if not found
    return;
  }
  ind2 = inString.indexOf(';', ind1 + 1);  //Find the first ';' after the colon
  if (ind2 == -1) {
    return;
  }
  c[0] = inString.substring(ind1 + 1, ind2).toFloat();  //Extract substring from ':' to the first ';' and convert it to float
  ind3 = inString.indexOf(';', ind2 + 1);  //Find the next ';'
  if (ind3 == -1) {
    return;
  }
  c[1] = inString.substring(ind2 + 1, ind3).toFloat();
  ind4 = inString.indexOf(';', ind3 + 1);
  if (ind4 == -1) {
    return;
  }
  c[2] = inString.substring(ind3 + 1, ind4).toFloat();
  ind5 = inString.indexOf(';', ind4 + 1);
  if (ind5 == -1) {
    return;
  }
  c[3] = inString.substring(ind4 + 1, ind5).toFloat();
  ind6 = inString.indexOf(';', ind5 + 1);
  if (ind6 == -1) {
    return;
  }
  c[4] = inString.substring(ind5 + 1, ind6).toFloat();
  ind7 = inString.indexOf(';', ind6 + 1);
  if (ind7 == -1) {
    return;
  }
  c[5] = inString.substring(ind6 + 1, ind7).toFloat();  
  ind8 = inString.indexOf(';', ind7 + 1);
  if (ind8 == -1) {
    return;
  }
  c[6] = inString.substring(ind7 + 1, ind8).toFloat(); 
  ind9 = inString.indexOf(';', ind8 + 1);
  if (ind9 == -1) {
    return;
  }
  c[7] = inString.substring(ind8 + 1, ind9).toFloat(); 

  ind10 = inString.indexOf(';', ind9 + 1);
  if (ind10 == -1) {
    return;
  }
  c[8] = inString.substring(ind9 + 1, ind10).toFloat(); 
}

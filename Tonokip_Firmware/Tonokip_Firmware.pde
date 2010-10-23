// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// with MrAlvin's extensions
// Licence: GPL

#define MOTHERBOARD 4 // See "pins.h" for details  (will not compile on ATmega168 - code base is too big)
                       // ATMEGA168 = 0, SANGUINO = 1, RepRap Gen3 MOTHERBOARD = 2, MEGA = 3, ATMEGA328 = 4, MEGA(MrAlvin) = 14, MEGA(Rapatan) = 15

#define FIRMWARE_VERSION "Tonokip 2010.10.12 MA v1.00.0005"
/* Notes about this version:
    Single pin control for Extruder heater (HEATER_0_PIN, TEMP_0_PIN) has been implemented
    Single pin control for Heated bed (HEATER_2_PIN, TEMP_2_PIN) has been implemented
    Physical Kill Pin has been implemented   -- untested
*/

#include "configuration.h"
#include "pins.h"
#include "ThermistorTable.h"
#include <EEPROM.h>

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0 -> G1
// G1  - Coordinated Movement X Y Z E
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - void - 
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M0   - Stop -  Finishes any moves left in command buffer, then shut down (command buffer not implemented yet)   -- untested
// M104 - Set target temp
// M105 - Read current temp
// M106 - Fan On (Sxx sets the voltage on the fan - Values are 0-12))   -- untested
// M107 - Fan off   -- untested
// M109 - Wait for current temp to reach target temp.
// M112 - Emergency Stop -  terminate immediately, turned off motors and heaters, and shut down NOW!    -- untested
// M115 - Get Firmware Version  -- untested
// M140 - Bed Temperature (Fast) = Set target Temperature (do not wait for it to be reached)

//Custom M Codes
// M70  - debugging mode on
// M71  - debugging mode off
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M86  - If Endstop is Not Activated then Abort Print. Specify X and/or Y
// M92  - Set axis_steps_per_unit (also called Calibration variables)- same syntax as G92.  Use M115  to read Calibration variables
// M201 - turn light on - (Sxxx sets the Light PWM. Values are 0-255)  -- untested
// M202 - turn light off  -- untested
// M312 - Save Calibration variables to EEPROM  -- untested
// M313 - Load Calibration variables from EEPROM  -- untested


//GCodes that wants to be implemented
// G28 - Move to Origin - so Home All button in RepSnapper will work


//Functions that wants to be improved
// MoveBuffer - will improve printing of small objects, and possibly more
// PID & PWM temperature control - will greatly improve control of thickness of extruded thread (should be operated within +/- 1*C)
// Propper calibration test bed for making a much more precise NTC table.
// Feedback on steppers - no more missed steps!! May also improve printable speeds

//Functions that could be improved/made
// M50 - change step mode, input X Y Z, syntax: M50 X1 Y1 Z4 (set x and y axes to full torque and use 4x microstepping on z)
// M301 - turn slave LED on
// M302 - turn slave LED off
// Manage slave temp controller
// Propper RS-485 (serial) communication using USARTs
// LCD display


// FIRMWARE OPTIONS  
boolean debugging = false; // use this to control parts of serial.print statements
int eepromAddr = 0; // this is the byte where the EEPROM functions will start writing

// Stepper Movement Variables
bool direction_x, direction_y, direction_z, direction_e;
unsigned long previous_micros=0, previous_micros_x=0, previous_micros_y=0, previous_micros_z=0, previous_micros_e=0, previous_millis_heater;
unsigned long x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take;
float destination_x =0.0, destination_y = 0.0, destination_z = 0.0, destination_e = 0.0;
float current_x = 0.0, current_y = 0.0, current_z = 0.0, current_e = 0.0;
float x_interval, y_interval, z_interval, e_interval; // for speed delay
float feedrate = 1500, next_feedrate;
float time_for_move;
long gcode_N, gcode_LastN;
bool relative_mode = false;     //Determines Absolute or Relative Coordinates
bool relative_mode_e = false;   //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

// comm variables
#define MAX_CMD_SIZE 256
char cmdbuffer[MAX_CMD_SIZE];
char serial_char;
int serial_count = 0;
boolean comment_mode = false;
char *strchr_pointer;   // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//manage heater variables
int heater_target_raw = 0;
int heater_current_raw;

int bed_target_raw = 0;
int bed_current_raw;

unsigned long previous_millis_heat=0;
unsigned long wait_heat_time = 100;    //we only need to check the heater 10 times a second

//Inactivity shutdown variables
unsigned long previous_millis_cmd=0;
unsigned long max_inactive_time = 0;


void setup() { 
  //Initialize Step Pins
  if(X_STEP_PIN > -1) pinMode(X_STEP_PIN,OUTPUT);
  if(Y_STEP_PIN > -1) pinMode(Y_STEP_PIN,OUTPUT);
  if(Z_STEP_PIN > -1) pinMode(Z_STEP_PIN,OUTPUT);
  if(E_STEP_PIN > -1) pinMode(E_STEP_PIN,OUTPUT);
  
  //Initialize Dir Pins
  if(X_DIR_PIN > -1) pinMode(X_DIR_PIN,OUTPUT);
  if(Y_DIR_PIN > -1) pinMode(Y_DIR_PIN,OUTPUT);
  if(Z_DIR_PIN > -1) pinMode(Z_DIR_PIN,OUTPUT);
  if(E_DIR_PIN > -1) pinMode(E_DIR_PIN,OUTPUT);

  //Steppers default to disabled.
  if(X_ENABLE_PIN > -1) if(!X_ENABLE_ON) digitalWrite(X_ENABLE_PIN,HIGH);
  if(Y_ENABLE_PIN > -1) if(!Y_ENABLE_ON) digitalWrite(Y_ENABLE_PIN,HIGH);
  if(Z_ENABLE_PIN > -1) if(!Z_ENABLE_ON) digitalWrite(Z_ENABLE_PIN,HIGH);
  if(E_ENABLE_PIN > -1) if(!E_ENABLE_ON) digitalWrite(E_ENABLE_PIN,HIGH);
  
  //Initialize Enable Pins
  if(X_ENABLE_PIN > -1) pinMode(X_ENABLE_PIN,OUTPUT);
  if(Y_ENABLE_PIN > -1) pinMode(Y_ENABLE_PIN,OUTPUT);
  if(Z_ENABLE_PIN > -1) pinMode(Z_ENABLE_PIN,OUTPUT);
  if(E_ENABLE_PIN > -1) pinMode(E_ENABLE_PIN,OUTPUT);

  if(HEATER_0_PIN > -1) pinMode(HEATER_0_PIN,OUTPUT);
  if(HEATER_1_PIN > -1) pinMode(HEATER_1_PIN,OUTPUT);
  if(HEATER_2_PIN > -1) pinMode(HEATER_2_PIN,OUTPUT);
  if(HEATER_3_PIN > -1) pinMode(HEATER_3_PIN,OUTPUT);
  
  //Initialize Control Pins
  if(FAN_PIN_0 > -1) pinMode(FAN_PIN_0,OUTPUT);  
  if(FAN_PIN_1 > -1) pinMode(FAN_PIN_1,OUTPUT); 
  if(KILL_PIN > -1) { pinMode(KILL_PIN,INPUT);  digitalWrite(KILL_PIN, HIGH); }       // turn on internal 20k pull-up resistor
  if(LIGHT_PIN_0 > -1) pinMode(LIGHT_PIN_0,OUTPUT);  
  if(LIGHT_PIN_1 > -1) pinMode(LIGHT_PIN_1,OUTPUT);  
  
  //MicroStep pins default to disabled (no micro stepping).
  if(MS1_PIN > -1) digitalWrite(MS1_PIN,LOW);
  if(MS2_PIN > -1) digitalWrite(MS2_PIN,LOW);
  if(MS3_PIN > -1) digitalWrite(MS3_PIN,LOW);
  
  //Initialize MicroStep pins
  if(MS1_PIN > -1) pinMode(MS1_PIN,OUTPUT);
  if(MS2_PIN > -1) pinMode(MS2_PIN,OUTPUT);
  if(MS3_PIN > -1) pinMode(MS3_PIN,OUTPUT);
  
  Serial.begin(HOST_BAUDRATE);
  Serial.println("start");
  
}


void loop()  {
  get_command();
  manage_heater();
  updateLCD();
  manage_inactivity(1); //shutdown if not receiving any new commands
  manage_kill_pin();
}

void updateLCD() {
}



inline void get_command() { 

  if( Serial.available() > 0 ) {
    serial_char = Serial.read();
    if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) )   {
      if(!serial_count) return; //if empty line
      cmdbuffer[serial_count] = 0; //terminate string
      Serial.print("Echo:");
      Serial.println(&cmdbuffer[0]);
      
      process_commands();
      
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
      //Serial.println("ok"); 
    }
    else  {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[serial_count++] = serial_char; 
    }
  }  
}


//#define code_num (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL))
//inline void code_search(char code) { strchr_pointer = strchr(cmdbuffer, code); }
inline float code_value() { return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)); }
inline long code_value_long() { return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10)); }
inline bool code_seen(char code_string[]) { return (strstr(cmdbuffer, code_string) != NULL); }  //Return True if the string was found

inline bool code_seen(char code)  {
  strchr_pointer = strchr(cmdbuffer, code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}



inline void process_commands()  {
  unsigned long codenum; //throw away variable
  
  if(code_seen('N'))  {
    gcode_N = code_value_long();
    if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer, "M110") == NULL) ) {
    //if(gcode_N != gcode_LastN+1 && !code_seen("M110") ) {   //Hmm, compile size is different between using this vs the line above even though it should be the same thing. Keeping old method.
      Serial.print("Serial Error: Line Number is not Last Line Number+1, Last Line:");
      Serial.println(gcode_LastN);
      FlushSerialRequestResend();
      return;
    }
    
    if(code_seen('*'))  {
      byte checksum = 0;
      byte count=0;
      while(cmdbuffer[count] != '*') checksum = checksum^cmdbuffer[count++];
     
      if( (int)code_value() != checksum) {
        Serial.print("Error: checksum mismatch, Last Line:");
        Serial.println(gcode_LastN);
        FlushSerialRequestResend();
        return;
      }
      //if no errors, continue parsing
    }
    else  {
      Serial.print("Error: No Checksum with line number, Last Line:");
      Serial.println(gcode_LastN);
      FlushSerialRequestResend();
      return;
    }
    
    gcode_LastN = gcode_N;
    //if no errors, continue parsing
  }
  else { // if we don't receive 'N' but still see '*'
    if(code_seen('*'))  {
      Serial.print("Error: No Line Number with checksum, Last Line:");
      Serial.println(gcode_LastN);
      return;
    }
  }

  //continues parsing only if we don't receive any 'N' or '*' or no errors if we do. :)
  
  if(code_seen('G'))  {
    switch((int)code_value())  {
      case 0: // G0 -> G1
      case 1: // G1
        get_coordinates(); // For X Y Z E F
        x_steps_to_take = abs(destination_x - current_x)*x_steps_per_unit;
        y_steps_to_take = abs(destination_y - current_y)*y_steps_per_unit;
        z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
        e_steps_to_take = abs(destination_e - current_e)*e_steps_per_unit;

        #define X_TIME_FOR_MOVE ((float)x_steps_to_take / (x_steps_per_unit*feedrate/60000000))
        #define Y_TIME_FOR_MOVE ((float)y_steps_to_take / (y_steps_per_unit*feedrate/60000000))
        #define Z_TIME_FOR_MOVE ((float)z_steps_to_take / (z_steps_per_unit*feedrate/60000000))
        #define E_TIME_FOR_MOVE ((float)e_steps_to_take / (e_steps_per_unit*feedrate/60000000))
        
        time_for_move = max(X_TIME_FOR_MOVE,Y_TIME_FOR_MOVE);
        time_for_move = max(time_for_move,Z_TIME_FOR_MOVE);
        time_for_move = max(time_for_move,E_TIME_FOR_MOVE);

        if(x_steps_to_take) x_interval = time_for_move/x_steps_to_take;
        if(y_steps_to_take) y_interval = time_for_move/y_steps_to_take;
        if(z_steps_to_take) z_interval = time_for_move/z_steps_to_take;
        if(e_steps_to_take) e_interval = time_for_move/e_steps_to_take;
        
        
        if(debugging == true) {
          Serial.print("destination_x: "); Serial.println(destination_x); 
          Serial.print("current_x: "); Serial.println(current_x); 
          Serial.print("x_steps_to_take: "); Serial.println(x_steps_to_take); 
          Serial.print("X_TIME_FOR_MVE: "); Serial.println(X_TIME_FOR_MOVE); 
          Serial.print("x_interval: "); Serial.println(x_interval); 
          Serial.println("");
          Serial.print("destination_y: "); Serial.println(destination_y); 
          Serial.print("current_y: "); Serial.println(current_y); 
          Serial.print("y_steps_to_take: "); Serial.println(y_steps_to_take); 
          Serial.print("Y_TIME_FOR_MVE: "); Serial.println(Y_TIME_FOR_MOVE); 
          Serial.print("y_interval: "); Serial.println(y_interval); 
          Serial.println("");
          Serial.print("destination_z: "); Serial.println(destination_z); 
          Serial.print("current_z: "); Serial.println(current_z); 
          Serial.print("z_steps_to_take: "); Serial.println(z_steps_to_take); 
          Serial.print("Z_TIME_FOR_MVE: "); Serial.println(Z_TIME_FOR_MOVE); 
          Serial.print("z_interval: "); Serial.println(z_interval); 
          Serial.println("");
          Serial.print("destination_e: "); Serial.println(destination_e); 
          Serial.print("current_e: "); Serial.println(current_e); 
          Serial.print("e_steps_to_take: "); Serial.println(e_steps_to_take); 
          Serial.print("E_TIME_FOR_MVE: "); Serial.println(E_TIME_FOR_MOVE); 
          Serial.print("e_interval: "); Serial.println(e_interval); 
          Serial.println("");
        }
        
        linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
        ClearToSend();
        return;
      case 4: // G4 dwell
        codenum = 0;
        if(code_seen('P')) codenum = code_value(); // milliseconds to wait
        if(code_seen('S')) codenum = code_value()*1000; // seconds to wait
        previous_millis_heater = millis();  // keep track of when we started waiting
        while((millis() - previous_millis_heater) < codenum ) manage_heater(); //manage heater until time is up
        break;
      case 28: //G28 - Home All
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        if(code_seen('X')) current_x = code_value();
        if(code_seen('Y')) current_y = code_value();
        if(code_seen('Z')) current_z = code_value();
        if(code_seen('E')) current_e = code_value();
        break;
        
    }
  }

  if(code_seen('M'))  {
    
    switch( (int)code_value() )   {
      case 0: // M0 - Stop
        kill(6);
        break;
      case 70: // M70, debugging mode turned on
        debugging = true;
        break;
      case 71: // M71, debugging mode turned off
        debugging = false;
        break;
      case 104: // M104 - Set target temp
        if (code_seen('S')) heater_target_raw = temp2analog(code_value(), THERMISTOR_ON_0);
        break;
      case 105: // M105 - Read target temp
        Serial.print("T:");
        Serial.print( analog2temp(analogRead(TEMP_0_PIN), THERMISTOR_ON_0) );  // Extruder temp
        Serial.print(" - ");
        Serial.println(analog2temp(analogRead(TEMP_2_PIN), THERMISTOR_ON_2) );  //Heated bed Temp
        if(!code_seen('N')) return;  // If M105 is sent from generated gcode, then it needs a response.
        break;
      case 106: // M106 - Fan On (Sxx sets the voltage on the fan)
         if(FAN_PIN_0 > -1) if (code_seen('S')) analogWrite(FAN_PIN_0, ((code_value()/12.0)*254 ));
        break;
      case 107: // M107 - Fan off
        if(FAN_PIN_0 > -1) analogWrite(FAN_PIN_0, 0);
        break;
      case 109: // M109 - Wait for heater to reach target.
        if (code_seen('S')) heater_target_raw = temp2analog(code_value(), THERMISTOR_ON_0);
        previous_millis_heater = millis(); 
        while(heater_current_raw < heater_target_raw) {
          if( (millis()-previous_millis_heater) > 1000 ) {  //Print Temp Reading every 1 second while heating up.
            Serial.print("T:");
            Serial.println( analog2temp(analogRead(TEMP_0_PIN), THERMISTOR_ON_0)); 
            previous_millis_heater = millis(); 
          }
          manage_heater();
        }
        break;
      case 112:
        kill(5);
        break;
      case 115: // M115 - Get Firmware Version
        Serial.print("Firmware Version: "); Serial.println(FIRMWARE_VERSION);
        Serial.print("x_steps_per_unit: "); Serial.println(x_steps_per_unit);
        Serial.print("y_steps_per_unit: "); Serial.println(y_steps_per_unit);
        Serial.print("z_steps_per_unit: "); Serial.println(z_steps_per_unit);
        Serial.print("e_steps_per_unit: "); Serial.println(e_steps_per_unit);
        Serial.print("max_feedrate: "); Serial.println(max_feedrate);
        Serial.println();
        break;
      case 140: // M140 - Bed Temperature (Fast)
        if (code_seen('S')) bed_target_raw = temp2analog(code_value(), THERMISTOR_ON_2);
        break;
      case 80: // M81 - ATX Power On
        if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,OUTPUT); //GND
        break;
      case 81: // M81 - ATX Power Off
        if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT); //Floating
        break;
      case 82: // M82  - Set E codes absolute (default)
        relative_mode_e = false;
        break;
      case 83:// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
        relative_mode_e = true;
        break;
      case 84: // M84  - Disable steppers until next move
        disable_x();
        disable_y();
        disable_z();
        disable_e();
        break;
      case 85: // M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
        code_seen('S');
        max_inactive_time = code_value()*1000; 
        break;
      case 86: // M86 If Endstop is Not Activated then Abort Print
        if(code_seen('X')) if( digitalRead(X_MIN_PIN) == ENDSTOPS_INVERTING ) kill(3);
        if(code_seen('Y')) if( digitalRead(Y_MIN_PIN) == ENDSTOPS_INVERTING ) kill(4);
        break;
      case 92: // M92  - Set axis_steps_per_unit - same syntax as G92
        if(code_seen('X')) x_steps_per_unit = code_value();
        if(code_seen('Y')) y_steps_per_unit = code_value();
        if(code_seen('Z')) z_steps_per_unit = code_value();
        if(code_seen('E')) e_steps_per_unit = code_value();
        break;
      case 201: // M201 - Light On (Sxxx sets the Light PWM. Values are 0-255)
        if(LIGHT_PIN_0 > -1) if (code_seen('S')) analogWrite(LIGHT_PIN_0, (code_value()));
        break;
      case 202: // M202 - Light off
        if(LIGHT_PIN_0 > -1) analogWrite(LIGHT_PIN_0, 0);
        break;
      case 312: // M312 - Save Calibration variables to EEPROM
        Save_Calibration_eeprom();
        break;
      case 313: // M313 - Load Calibration variables from EEPROM
        Load_Calibration_eeprom();
        break;
    }
  }
  ClearToSend();
}

inline void FlushSerialRequestResend() {
  char cmdbuffer[100]="Resend:";
  ltoa(gcode_LastN+1, cmdbuffer+7, 10);
  Serial.flush();
  Serial.println(cmdbuffer);
  ClearToSend();
}

inline void ClearToSend() {
  previous_millis_cmd = millis();
  Serial.println("ok"); 
}

inline void get_coordinates() {
  if(code_seen('X')) destination_x = (float)code_value() + relative_mode*current_x;
  else destination_x = current_x;                                                       //Are these else lines really needed?
  if(code_seen('Y')) destination_y = (float)code_value() + relative_mode*current_y;
  else destination_y = current_y;
  if(code_seen('Z')) destination_z = (float)code_value() + relative_mode*current_z;
  else destination_z = current_z;
  if(code_seen('E')) destination_e = (float)code_value() + (relative_mode_e || relative_mode)*current_e;
  else destination_e = current_e;
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  
  //Find direction
  if(destination_x >= current_x) direction_x=1;
  else direction_x=0;
  if(destination_y >= current_y) direction_y=1;
  else direction_y=0;
  if(destination_z >= current_z) direction_z=1;
  else direction_z=0;
  if(destination_e >= current_e) direction_e=1;
  else direction_e=0;
  
  
  if (min_software_endstops) {
    if (destination_x < 0) destination_x = 0.0;
    if (destination_y < 0) destination_y = 0.0;
    if (destination_z < 0) destination_z = 0.0;
  }

  if (max_software_endstops) {
    if (destination_x > X_MAX_LENGTH) destination_x = X_MAX_LENGTH;
    if (destination_y > Y_MAX_LENGTH) destination_y = Y_MAX_LENGTH;
    if (destination_z > Z_MAX_LENGTH) destination_z = Z_MAX_LENGTH;
  }
  
  if(feedrate > max_feedrate) feedrate = max_feedrate;
}

void linear_move(unsigned long x_steps_remaining, unsigned long y_steps_remaining, unsigned long z_steps_remaining, unsigned long e_steps_remaining) { // make linear move with preset speeds and destinations, see G0 and G1
  //Determine direction of movement
  if (destination_x > current_x) digitalWrite(X_DIR_PIN,!INVERT_X_DIR);
  else digitalWrite(X_DIR_PIN,INVERT_X_DIR);
  if (destination_y > current_y) digitalWrite(Y_DIR_PIN,!INVERT_Y_DIR);
  else digitalWrite(Y_DIR_PIN,INVERT_Y_DIR);
  if (destination_z > current_z) digitalWrite(Z_DIR_PIN,!INVERT_Z_DIR);
  else digitalWrite(Z_DIR_PIN,INVERT_Z_DIR);
  if (destination_e > current_e) digitalWrite(E_DIR_PIN,!INVERT_E_DIR);
  else digitalWrite(E_DIR_PIN,INVERT_E_DIR);
  
  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  if(x_steps_remaining) enable_x();
  if(y_steps_remaining) enable_y();
  if(z_steps_remaining) enable_z();
  if(e_steps_remaining) enable_e();

  if(X_MIN_PIN > -1) if(!direction_x) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) x_steps_remaining=0;
  if(Y_MIN_PIN > -1) if(!direction_y) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) y_steps_remaining=0;
  if(Z_MIN_PIN > -1) if(!direction_z) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) z_steps_remaining=0;
  
  previous_millis_heater = millis();

  //while(x_steps_remaining > 0 || y_steps_remaining > 0 || z_steps_remaining > 0 || e_steps_remaining > 0) // move until no more steps remain
  while(x_steps_remaining + y_steps_remaining + z_steps_remaining + e_steps_remaining > 0) {  // move until no more steps remain
    if(x_steps_remaining) {
      if ((micros()-previous_micros_x) >= x_interval) { do_x_step(); x_steps_remaining--; }
      if(X_MIN_PIN > -1) if(!direction_x) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) x_steps_remaining=0;
    }
    
    if(y_steps_remaining) {
      if ((micros()-previous_micros_y) >= y_interval) { do_y_step(); y_steps_remaining--; }
      if(Y_MIN_PIN > -1) if(!direction_y) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) y_steps_remaining=0;
    }
    
    if(z_steps_remaining) {
      if ((micros()-previous_micros_z) >= z_interval) { do_z_step(); z_steps_remaining--; }
      if(Z_MIN_PIN > -1) if(!direction_z) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) z_steps_remaining=0;
    }    
    
    if(e_steps_remaining) if ((micros()-previous_micros_e) >= e_interval) { do_e_step(); e_steps_remaining--; }
    
    if( (millis() - previous_millis_heater) >= 500 ) {
      manage_heater();
      previous_millis_heater = millis();
      
      manage_inactivity(2);
    }
  }
  
  if(DISABLE_X) disable_x();
  if(DISABLE_Y) disable_y();
  if(DISABLE_Z) disable_z();
  if(DISABLE_E) disable_e();
  
  // Update current position partly based on direction, we probably can combine this with the direction code above...
  if (destination_x > current_x) current_x = current_x + x_steps_to_take/x_steps_per_unit;
  else current_x = current_x - x_steps_to_take/x_steps_per_unit;
  if (destination_y > current_y) current_y = current_y + y_steps_to_take/y_steps_per_unit;
  else current_y = current_y - y_steps_to_take/y_steps_per_unit;
  if (destination_z > current_z) current_z = current_z + z_steps_to_take/z_steps_per_unit;
  else current_z = current_z - z_steps_to_take/z_steps_per_unit;
  if (destination_e > current_e) current_e = current_e + e_steps_to_take/e_steps_per_unit;
  else current_e = current_e - e_steps_to_take/e_steps_per_unit;
}


inline void do_x_step() {
  digitalWrite(X_STEP_PIN, HIGH);
  previous_micros_x = micros();
  //delayMicroseconds(3);
  digitalWrite(X_STEP_PIN, LOW);
}

inline void do_y_step() {
  digitalWrite(Y_STEP_PIN, HIGH);
  previous_micros_y = micros();
  //delayMicroseconds(3);
  digitalWrite(Y_STEP_PIN, LOW);
}

inline void do_z_step() {
  digitalWrite(Z_STEP_PIN, HIGH);
  previous_micros_z = micros();
  //delayMicroseconds(3);
  digitalWrite(Z_STEP_PIN, LOW);
}

inline void do_e_step() {
  digitalWrite(E_STEP_PIN, HIGH);
  previous_micros_e = micros();
  //delayMicroseconds(3);
  digitalWrite(E_STEP_PIN, LOW);
}

inline void disable_x() { if(X_ENABLE_PIN > -1) digitalWrite(X_ENABLE_PIN,!X_ENABLE_ON); }
inline void disable_y() { if(Y_ENABLE_PIN > -1) digitalWrite(Y_ENABLE_PIN,!Y_ENABLE_ON); }
inline void disable_z() { if(Z_ENABLE_PIN > -1) digitalWrite(Z_ENABLE_PIN,!Z_ENABLE_ON); }
inline void disable_e() { if(E_ENABLE_PIN > -1) digitalWrite(E_ENABLE_PIN,!E_ENABLE_ON); }
inline void  enable_x() { if(X_ENABLE_PIN > -1) digitalWrite(X_ENABLE_PIN, X_ENABLE_ON); }
inline void  enable_y() { if(Y_ENABLE_PIN > -1) digitalWrite(Y_ENABLE_PIN, Y_ENABLE_ON); }
inline void  enable_z() { if(Z_ENABLE_PIN > -1) digitalWrite(Z_ENABLE_PIN, Z_ENABLE_ON); }
inline void  enable_e() { if(E_ENABLE_PIN > -1) digitalWrite(E_ENABLE_PIN, E_ENABLE_ON); }


inline void manage_heater(){
  if( (millis()-previous_millis_heat) >  wait_heat_time )    {              //we only need to check the heater 10 times a second  

      previous_millis_heat = millis();
      
      //Extruder
      heater_current_raw = analogRead(TEMP_0_PIN);                          // If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
      if(THERMISTOR_ON_0) heater_current_raw = 1023 - heater_current_raw;   // this switches it up so that the reading appears lower than target for the control logic.
      
      if(heater_current_raw >= heater_target_raw) digitalWrite(HEATER_0_PIN,LOW);
      else digitalWrite(HEATER_0_PIN,HIGH);
    
      //Heated bed 
      if (MANAGE_HEATED_BED){
          bed_current_raw = analogRead(TEMP_2_PIN);                          // If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
          if(THERMISTOR_ON_2) bed_current_raw = 1023 - bed_current_raw;      // this switches it up so that the reading appears lower than target for the control logic.
            
          if(bed_current_raw >= bed_target_raw) digitalWrite(HEATER_2_PIN,LOW);
          else digitalWrite(HEATER_2_PIN,HIGH);
      }
  }
  manage_kill_pin();
}

// Takes temperature value as input and returns corresponding analog value from RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inversing the logic from a portion of getTemperature() in FiveD RepRap firmware.
float temp2analog(int celsius, boolean use_thermistor) {
  if(use_thermistor) {
    int raw = 0;
    byte i;
    
    for (i=1; i<NUMTEMPS; i++)   {
      if (temptable[i][1] < celsius)   {
        raw = temptable[i-1][0] + 
          (celsius - temptable[i-1][1]) * 
          (temptable[i][0] - temptable[i-1][0]) /
          (temptable[i][1] - temptable[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) raw = temptable[i-1][0];

    return 1023 - raw;
  } else {
    return celsius * (1024.0/(5.0*100.0));
  }
}

// Derived from RepRap FiveD extruder::getTemperature()
float analog2temp(int raw, boolean use_thermistor) {
  if(use_thermistor) {
    int celsius = 0;
    byte i;

    for (i=1; i<NUMTEMPS; i++)
    {
      if (temptable[i][0] > raw)
      {
        celsius  = temptable[i-1][1] + 
          (raw - temptable[i-1][0]) * 
          (temptable[i][1] - temptable[i-1][1]) /
          (temptable[i][0] - temptable[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) celsius = temptable[i-1][1];

    return celsius;
    
  } else {
    return raw * ((5.0*100.0)/1024.0);
  }
}



// EEPROM functions for handling more than byte size types, see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290
template <class T> int EEPROM_writeAnything(int ee, const T& value)  {
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++) {  EEPROM.write(ee++, *p++); }
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)  {
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++) { *p++ = EEPROM.read(ee++);  }
  return i;
}

inline void Save_Calibration_eeprom()  {
  int addr = eepromAddr;
  addr += EEPROM_writeAnything(addr, x_steps_per_unit);
  addr += EEPROM_writeAnything(addr, y_steps_per_unit);
  addr += EEPROM_writeAnything(addr, e_steps_per_unit);
  addr += EEPROM_writeAnything(addr, max_feedrate);  
}

inline void Load_Calibration_eeprom()  {
  int addr = eepromAddr;
  addr += EEPROM_readAnything(addr, x_steps_per_unit);
  addr += EEPROM_readAnything(addr, y_steps_per_unit);
  addr += EEPROM_readAnything(addr, e_steps_per_unit);
  addr += EEPROM_readAnything(addr, max_feedrate);  
}



inline void manage_kill_pin() {  if (KILL_PIN > -1) if (digitalRead(KILL_PIN) == LOW)  kill(7); }

inline void kill(byte debug) {
  if(HEATER_0_PIN > -1) digitalWrite(HEATER_0_PIN,LOW);
  if(HEATER_1_PIN > -1) digitalWrite(HEATER_1_PIN,LOW);
  if(HEATER_2_PIN > -1) digitalWrite(HEATER_2_PIN,LOW);
  if(HEATER_3_PIN > -1) digitalWrite(HEATER_3_PIN,LOW);
  
  disable_x;
  disable_y;
  disable_z;
  disable_e;
  
  if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
  
  while(1) {
    switch(debug) {
      case 1: Serial.print("Inactivity Shutdown, Last Line: "); break;
      case 2: Serial.print("Linear Move Abort, Last Line: "); break;
      case 3: Serial.print("Homing X Min Stop Fail, Last Line: "); break;
      case 4: Serial.print("Homing Y Min Stop Fail, Last Line: "); break;
      case 5: Serial.print("User Selected Emergency Shutdown, Last Line: "); break;
      case 6: Serial.print("User Selected Stop and Shutdown, Last Line: "); break;
      case 7: Serial.print("Kill_pin Stop and Shutdown, Last Line: "); break;
    } 
    Serial.println(gcode_LastN);
    delay(5000); // 5 Second delay
  }
}

inline void manage_inactivity(byte debug) { if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(debug); }

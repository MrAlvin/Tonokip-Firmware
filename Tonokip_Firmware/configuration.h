#ifndef PARAMETERS_H
#define PARAMETERS_H

// NO RS485/EXTRUDER CONTROLLER SUPPORT
// PLEASE VERIFY PIN ASSIGNMENTS FOR YOUR CONFIGURATION!!!!!!!


// THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!
//const bool USE_THERMISTOR = true; //Set to false if using thermocouple - no longer used globally.
                                    //as of 1.00.0005 Set thermistor use under each heater definition in pins.h!!!


                                    
const int EXTRUDER_COUNT = 1;
const boolean  MANAGE_HEATED_BED = true;


#define SLAVE_TEMP_CONTROL 0 // 1 for on, 0 for off
// if you want to use this hook up a 2nd Arduino to the master with TX of the slave connected to RX of the master, and RX of the slave to TX of the master
// the temperature signals should also be going into the slave with the pin declarations defined with respect to the slave Arduino



// Calibration formulas
// e_extruded_steps_per_mm = e_feedstock_steps_per_mm * (desired_extrusion_diameter^2 / feedstock_diameter^2)
// new_axis_steps_per_mm = previous_axis_steps_per_mm * (test_distance_instructed/test_distance_traveled)
// units are in millimeters or whatever length unit you prefer: inches,football-fields,parsecs etc

//Calibration variables for 
//float x_steps_per_unit = 10.047; //Half step setting of X (default Mendel design)
//float y_steps_per_unit = 10.047; //Half step setting of Y (default Mendel design)
//float z_steps_per_unit = 833.398;//Half step setting of Z (default Mendel design)

float x_steps_per_unit = 5.0235;  //Full step setting of X
float y_steps_per_unit = 5.0235;  //Full step setting of Y
float z_steps_per_unit = 416.699; //Full step setting of Y

float e_steps_per_unit = 10.0;
float max_feedrate = 3000;

//For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
const bool X_ENABLE_ON = 0;
const bool Y_ENABLE_ON = 0;
const bool Z_ENABLE_ON = 0;
const bool E_ENABLE_ON = 0;

//Disables axis when it's not being used.
const bool DISABLE_X = false;
const bool DISABLE_Y = false;
const bool DISABLE_Z = true;
const bool DISABLE_E = false;             

const bool INVERT_X_DIR = true;
const bool INVERT_Y_DIR = false;
const bool INVERT_Z_DIR = true;
const bool INVERT_E_DIR = true;

//Endstop Settings
const bool ENDSTOPS_INVERTING = false;
const bool min_software_endstops = false; //If true, axis won't move to coordinates less than zero.
const bool max_software_endstops = true;  //If true, axis won't move to coordinates greater than the defined lengths below.
const int X_MAX_LENGTH = 200;
const int Y_MAX_LENGTH = 200;
const int Z_MAX_LENGTH = 120;

#define HOST_BAUDRATE 100000
#define SLAVE_BAUDRATE 19200     
#define LCD_BAUDRATE 19200       

#endif

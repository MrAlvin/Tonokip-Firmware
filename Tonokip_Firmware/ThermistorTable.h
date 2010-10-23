#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

//
// Thermistor notes (sept. 2010): 
//
// FiveD_GCode thermistor program code is found in file ../FiveD_GCode_Interpreter/Extruder/Extruder.pde -> tab:temperature.h 
//  where definition for several different NTCs can also be found.
// RepRap Thermistor Wiki page is at: http://reprap.org/wiki/Thermistor  and  http://make.rrrf.org/ts
// For formulas of NTC see also: http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html
//
// To find values for your Thermistor, see http://www.reprap.org/wiki/MeasuringThermistorBeta
//
// Python script to generate a "temptable" 
//  is here: http://reprap.svn.sourceforge.net/viewvc/reprap/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py?view=markup&pathrev=3448
//  or here:  http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py
//  or here:  https://reprap.svn.sourceforge.net/svnroot/reprap/trunk/mendel/firmware/createTemperatureLookup.py (seems to be the newest one)


/*
For thermistor:  www.Rapidonline.com - NTC part no. 104GT-1  - Order code: 61-0452
  More about Thermistor lookup table information for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts) 
  Table is made by hand, using a test setup and with constantan heater and a K-type referance sensor and an IR thermometer.


#define NUMTEMPS 30
short temptable[NUMTEMPS][2] = {
   {17, 280},
   {21, 270},
   {25, 260},
   {29, 250},
   {34, 240},
   {41, 230},
   {49, 220},
   {60, 210},
   {71, 200},
   {88, 190},
   {107, 180},
   {132, 170},
   {166, 160},
   {201, 150},
   {247, 140},
   {297, 130},
   {359, 120},
   {424, 110},
   {509, 100},
   {588, 90},
   {665, 80},
   {740, 70},
   {835, 60},
   {885, 50},
   {928, 40},
   {953, 30},
   {978, 20},
   {984, 10},
   {996, 1},
   {1006, 0}
};


*/

//   "The original" RepRap Thermistor table
// EPCOS 100K Thermistor (B57540G0104F000) - RS 528-8592
// Commandline to run the python script, is as follows:
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4066 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 4066
// max adc: 1023


#define NUMTEMPS 20
short temptable[NUMTEMPS][2] = {
   {1, 841},
   {54, 255},
   {107, 209},
   {160, 184},
   {213, 166},
   {266, 153},
   {319, 142},
   {372, 132},
   {425, 124},
   {478, 116},
   {531, 108},
   {584, 101},
   {637, 93},
   {690, 86},
   {743, 78},
   {796, 70},
   {849, 61},
   {902, 50},
   {955, 34},
   {1008, 3}
};


#endif

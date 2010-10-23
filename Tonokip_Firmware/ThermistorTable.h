#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

//
// Thermistor notes (sept. 2010) by MrAlvin 
//
// FiveD_GCode thermistor program code is found in file ../FiveD_GCode_Interpreter/Extruder/Extruder.pde -> tab:temperature.h 
// where definition for several different NTCs can also be found.
//
// - RepRap Thermistor Wiki page is at: http://reprap.org/wiki/Thermistor  and  http://make.rrrf.org/ts
// - For formulas of NTC see also: http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html
//
// - To find values for your particular type of Thermistor, see http://www.reprap.org/wiki/MeasuringThermistorBeta
//
// Python script to generate a "temptable" 
//    is here:  http://reprap.svn.sourceforge.net/viewvc/reprap/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py?view=markup&pathrev=3448
//    or here:  http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py
//    or here:  https://reprap.svn.sourceforge.net/svnroot/reprap/trunk/mendel/firmware/createTemperatureLookup.py (seems to be the newest one)

/**********************************************************************
 * Thermistor Table
 **********************************************************************
  For thermistor:  www.Rapidonline.com - NTC part no. 104GT-1  - Order code: 61-0452
  More about Thermistor lookup table information for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts) 
*/


#define NUMTEMPS 30
short temptable[NUMTEMPS][2] = {
   {7,355},
   {14,300},
   {20,280},
   {27,260},
   {37,240},
   {54,220},
   {59,215},
   {64,210},
   {71,205},
   {78,200},
   {85,195},
   {95,190},
   {105,185},
   {116,180},
   {129,175},
   {142,170},
   {158,165},
   {174,160},
   {193,155},
   {213,150},
   {238,145},
   {271,140},
   {385,120},
   {529,100},
   {681,80},
   {814,60},
   {904,40},
   {946,25},
   {974,10},
   {992,0}
};




// "The original" RepRap Thermistor table
//     EPCOS 100K Thermistor (B57540G0104F000) - RS 528-8592
//     Commandline to run the python script, is as follows:
//     ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4066 --max-adc=1023
//     r0: 100000
//     t0: 25
//     r1: 0
//     r2: 4700
//     beta: 4066
//     max adc: 1023

/*
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
*/

#endif

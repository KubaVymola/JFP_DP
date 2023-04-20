//==============================================================================
// demo_sequence.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef DEMOSEQUENCE_H
#define DEMOSEQUENCE_H

/**
 * A sequence of demo manoeuvres, The macro DEMO_SEQ in defines.h must be defined to make make
 * function called from the control_loop function
*/
void demo_sequence();

#endif
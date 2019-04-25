#ifndef _GRASHOF_CONDITION_
#define _GRASHOF_CONDITION_

#include "Simbody.h"

// Evaluation of Grashof condition, given the bar lenghts [ground, input, floating, output]. 
// This function determines if fourbar is double crank, double rocker or crank-rocker.
// It also determines if input bar is a crank and can fully rotate, returning a bool.

bool evaluateGrashof(std::vector<double>&);

#endif
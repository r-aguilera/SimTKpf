#ifndef _PF_UTILITIES_
#define _PF_UTILITIES_

#include "Simbody.h"
#include "Particle_Classes.h"

// Function to make a single State progress.
void advance(SimTK::State&, SimTK::TimeStepper&, const double);

// Return the exponent of a gaussian distribution function evaluated at x, given the mean and standart deviation s: 
double NormalProb(const double, const double, const double);

#endif
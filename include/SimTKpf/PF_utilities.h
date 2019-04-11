#ifndef _PF_UTILITIES_
#define _PF_UTILITIES_

#include "Simbody.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/Measuring_Instrument.h"
#include "SimTKpf/Simbody_Instrument.h"
#include "SimTKpf/Stopwatch.h"

// Function to make a single State progress.
void advance(SimTK::State&, SimTK::TimeStepper&, const double);

// Return the exponent of a gaussian distribution function evaluated at x, given the mean and standart deviation s: 
double NormalProb(const double, const double, const double);

// Return the given angle, expressed in the range [0, 2*Pi). Input/Output in radians!
inline double to2Pi(double Angle) {

	bool isNegative = Angle < 0;
	double Output_Angle = fmod(Angle, 2 * SimTK::Pi);
	if (isNegative) Output_Angle += 2 * SimTK::Pi;

	return Output_Angle;
}

// Return a integer to be used as a seed in random functions. Nanoseconds from last boot are turn into a integer. Updates every 0.001 ms.
inline int getSeed() {
	timespec ts;
	long long nanoseconds;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	nanoseconds = SimTK::timespecToNs(ts);
	
	return static_cast<int>(nanoseconds);
}

#endif
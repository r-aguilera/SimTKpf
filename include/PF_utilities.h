#ifndef _PF_UTILITIES_
#define _PF_UTILITIES_

#include "Simbody.h"
#include "Particle_Classes.h"

// Function to make a single State progress.
void advance(SimTK::State&, SimTK::TimeStepper&, const double);

// Return the exponent of a gaussian distribution function evaluated at x, given the mean and standart deviation s: 
double NormalProb(const double, const double, const double);

// Return the given angle, expressed in the range [0, 2*Pi). Input/Output in radians!
inline int getSeed();
inline double to2Pi(double Angle) {

	bool isNegative = Angle < 0;
	double Output_Angle = fmod(Angle, 2 * SimTK::Pi);
	if (isNegative) Output_Angle += 2 * SimTK::Pi;

	return Output_Angle;
}

// Return a integer to be used as a seed in random functions. Said integer contains the seconds elapsed since 1/1/1970.
inline int getSeed() {
	timespec ts;
	long long nanoseconds;
	double seconds;

	clock_gettime(CLOCK_REALTIME, &ts);
	nanoseconds = SimTK::timespecToNs(ts);
	seconds = SimTK::nsToSec(nanoseconds);

	return int(seconds);
}

#endif
#ifndef _GYROSCOPE_
#define _GYROSCOPE_

#include "Simbody.h"
#include "Measuring_Instrument.h"

// Class representing a virtual gyroscope measuring angular velocity
// in the first not-ground link of a four bar linkage

class Gyroscope : public Measuring_Instrument {
public:
	// Inherited constructor given the lenght of the ground bar
	Gyroscope(const SimTK::MultibodySystem&, const SimTK::SimbodyMatterSubsystem&, SimTK::State&, double, double);

	void measure() override;

private:
	double m_GroundBarLenght;
};

#endif
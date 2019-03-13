#ifndef _MEASURING_INSTRUMENT_
#define _MEASURING_INSTRUMENT_

// General class for virtual measuring instruments:

#include "Simbody.h"

class Measuring_Instrument {
public:
	// This constructor use parameters needed for any virtual instrument
	Measuring_Instrument(const SimTK::MultibodySystem&, const SimTK::SimbodyMatterSubsystem&,
		SimTK::State&, double);

	virtual void measure()=0;	// Update instrument reading. Pure virtual, must be overriden
				
	static void setGlobalSeed(int);	// Give RNG a new eed

	const double read();	// Return instrument reading

protected:
	const SimTK::SimbodyMatterSubsystem& m_matter;
	const SimTK::MultibodySystem& m_system;
	double m_StdDev;
	double reading;
	SimTK::State& rf_state;
	static SimTK::Random::Gaussian RNG;	

};

#endif
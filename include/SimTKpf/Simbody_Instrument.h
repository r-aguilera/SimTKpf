#ifndef _SIMBODY_INSTRUMENT_
#define _SIMBODY_INSTRUMENT_

#include "Simbody.h"
#include "SimTKpf/Measuring_Instrument.h"

// Class for not-ideal measuring instruments given Simbody systems and a State:

class Simbody_Instrument : public Measuring_Instrument {
public:
	Simbody_Instrument(const SimTK::MultibodySystem&, const SimTK::SimbodyMatterSubsystem&,
		SimTK::State&, double);
			
	static void setGlobalSeed(int);	// Give RNG a new seed

	const double read() override;

protected:
	const SimTK::SimbodyMatterSubsystem& m_matter;
	const SimTK::MultibodySystem& m_system;
	double m_StdDev;
	SimTK::State& rf_state;
	static SimTK::Random::Gaussian RNG;	
};

#endif
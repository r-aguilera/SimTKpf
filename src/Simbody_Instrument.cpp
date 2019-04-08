#include "Simbody.h"
#include "Measuring_Instrument.h"
#include "Simbody_Instrument.h"

Simbody_Instrument::Simbody_Instrument(const SimTK::MultibodySystem& system, const SimTK::SimbodyMatterSubsystem& matter,
		SimTK::State& RefState, double StdDev = 0) : m_system(system), m_matter(matter), rf_state(RefState), m_StdDev(StdDev) {}

void Simbody_Instrument::setGlobalSeed(int seed) { RNG.setSeed(seed); }

const double Simbody_Instrument::read(){
	RNG.setStdDev(m_StdDev);
	return reading + RNG.getValue();
}

SimTK::Random::Gaussian Simbody_Instrument::RNG;
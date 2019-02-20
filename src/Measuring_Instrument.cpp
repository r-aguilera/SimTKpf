#include "Simbody.h"
#include "Measuring_Instrument.h"

Measuring_Instrument::Measuring_Instrument(const SimTK::MultibodySystem& system, const SimTK::SimbodyMatterSubsystem& matter,
		SimTK::State& RefState, double StdDev = 0) : m_system(system), m_matter(matter), rf_state(RefState), m_StdDev(StdDev) {}

void Measuring_Instrument::setGlobalSeed(int seed) { RNG.setSeed(seed); }

const double Measuring_Instrument::read(){
	RNG.setStdDev(m_StdDev);
	return reading + RNG.getValue();
}

SimTK::Random::Gaussian Measuring_Instrument::RNG;
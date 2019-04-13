#ifndef _PARTICLE_FILTER_
#define _PARTICLE_FILTER_

#include "Simbody.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/Simbody_Instrument.h"
#include "SimTKpf/PF_utilities.h"

// Auxiliary enumeration class for accesing a single particle filter option:
enum class PF_Options_index {
	PARTICLE_NUMBER,
	SIMULATION_TIME_STEP,
	SENSOR_STDDEV,
	SENSOR_STDDEV_MOD,
	MOTION_STDDEV,
	RESAMPLE_STDDEV
};

// Options class for particle filter algorithm:
class PF_Options {
public:
	PF_Options();
	PF_Options(const std::size_t, const double, const double, const double, const double, const double);

	void setOptions(const std::size_t, const double, const double, const double, const double, const double);

	void setOption(PF_Options_index, double);		// Write single option double value
	void setOption(PF_Options_index, std::size_t);	// Write single option size_t value

	const std::size_t getOption(PF_Options_index);	// Return single option as read-only size_t value
	const double getOption(PF_Options_index);		// Return single option as read-only double value
	
	double& updOption(PF_Options_index);			// Return single option as writable double value
	std::size_t& updOption(PF_Options_index);		// Return single option as writable size_t value

private:
	std::size_t PARTICLE_NUMBER;
	double SIMULATION_TIME_STEP;
	double SENSOR_STDDEV;
	double SENSOR_STDDEV_MOD;
	double MOTION_STDDEV;
	double RESAMPLE_STDDEV;	
};

// Main SimTKpf class, for estimation based on particle filter algorithm:
class ParticleFilter {
public:
	ParticleFilter(PF_Options&);	// Constructor given his PF_Options

	// Advance particles state and add noise.
	void updateStates(ParticleList&, SimTK::TimeStepper&, SimTK::Assembler&, double, double);

	// Make a prediction and update weights accordingly.
	void updateWeights(ParticleList&, double, std::vector<Simbody_Instrument>*, double);

	// Replace particles according to their weights, passing ParticleList as argument.
	void resample(ParticleList&);
private:
	ParticleList Particles;
	PF_Options Filter_Options;
};


#endif

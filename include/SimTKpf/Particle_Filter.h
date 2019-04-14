#ifndef _PARTICLE_FILTER_
#define _PARTICLE_FILTER_

#include "Simbody.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/Simbody_Instrument.h"
#include "SimTKpf/PF_utilities.h"

// Auxiliary enumeration class for accesing a single particle filter option:
enum class PF_Options_index {
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
	// 1 - Time step between two filter iterations
	// 2 - Standard deviation of Ground Truth sensor's noise 
	// 3 - Decimal who modifies the sensor standard deviation during updating stage
	// 4 - Standard deviation of noise added in prediction stage
	// 5 - Standard deviation of noise added after the resampling stage
	PF_Options(double, double, double, double, double);

	void setOptions(double, double, double, double, double);
	void operator =(PF_Options);

	void setOption(PF_Options_index, double);		// Write single option value
	const double getOption(PF_Options_index) const;	// Return single option as read-only double value
	double& updOption(PF_Options_index);			// Return single option as writable double value

private:
	double SIMULATION_TIME_STEP;
	double SENSOR_STDDEV;
	double SENSOR_STDDEV_MOD;
	double MOTION_STDDEV;
	double RESAMPLE_STDDEV;	
};

// Main SimTKpf class, for estimation based on particle filter algorithm:
class ParticleFilter {
public:
	// Constructor given the particle number and his PF_Options
	ParticleFilter(std::size_t, PF_Options&);

	void setOptions(PF_Options);			// Write filter options values
	const PF_Options getOptions() const;	// Return filter options as read-only PF_Options class
	PF_Options& updOptions();				// Return filter options as writable PF_Options class

	const ParticleList getParticleList() const;	// Return particles as read-only ParticleList class
	ParticleList& updParticleList();			// Return particles as writable ParticleList class

	const std::vector<ParticleDynState> getParticleVec() const;	// Return particles as read-only std::vector
	std::vector<ParticleDynState>& updParticleVec();			// Return particles as writable std::vector

	std::size_t getParticleNumber() const;	// Return particle number value

	void setEqualWeights();			// Set equal logarithmic weights to all particles
	void setEqualLinearWeights();	// Set equal linear weights that sum 1 to all particles

	void normalizeWeights();		// Normalize particles' logarithmic weights
	void normalizeLinearWeights();	// Make the particles' weight distribution a probability distribution function (Sum must be 1):

	void calculateESS();			// Calculate the Effective Sample Size (ESS) of the particle list
	double getESS() const;			// Return the Effective Sample Size (ESS) value

	// Advance particles state and add noise.
	void updateStates(SimTK::TimeStepper&, SimTK::Assembler&);

	// Make a prediction and update weights accordingly.
	void updateWeights(std::vector<Simbody_Instrument>*);

	// Replace particles according to their weights.
	void resample();

private:
	ParticleList Particles;
	PF_Options& Filter_Options;
};


#endif

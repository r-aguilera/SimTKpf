#ifndef _PARTICLE_FILTER_
#define _PARTICLE_FILTER_

#include "Simbody.h"
#include "SimTKpf/PF_Options.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/Simbody_Instrument.h"
#include "SimTKpf/PF_utilities.h"

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
	template<class customInstrument>
	void updateWeights(double, std::vector<customInstrument>&);

	// Replace particles according to their weights.
	void resample();

private:
	ParticleList Particles;
	PF_Options& Filter_Options;
};

// Template Implementation
template<class customInstrument>
void ParticleFilter::updateWeights(double GroundTruthInstr_reading, std::vector<customInstrument>& InstrVec) {

	double Instrument_StdDev = Filter_Options.getOption(PF_Options_index::SENSOR_STDDEV);
	double Modified_Instrument_StdDev = Instrument_StdDev * Filter_Options.getOption(PF_Options_index::SENSOR_STDDEV_MOD);
	double belief;

	for (std::size_t i = 0; i < Particles.size(); i++) {
		InstrVec[i].measure();										// Update instrument reading
		belief = InstrVec[i].read();								// Make the prediction
		Particles[i].updWeight() += LogNormalProb(belief,			// Update weights
			GroundTruthInstr_reading, Modified_Instrument_StdDev);
	}
	Particles.normalizeWeights();
}

#endif

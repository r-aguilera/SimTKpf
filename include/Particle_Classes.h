#ifndef _PARTICLE_CLASSES_
#define _PARTICLE_CLASSES_

#include "Simbody.h"

// PF particles will belong to this class:

class ParticleDynState {
public:
	ParticleDynState();	// Default constructor. State will keep empty

	ParticleDynState(const SimTK::State&, double);	// Constructor given a state. Weight will be zero if unspecified

	void operator =(const ParticleDynState&);	// Operator to copy particles

	bool operator <(const ParticleDynState&);	// Operator necessary to sort particles

	void setWeight(const double&);			// Set weight value
	const double getWeight() const;			// Get read-only weight value
	double& updWeight();					// Get writable weight value

	void setState(const SimTK::State&);		// Set State
	const SimTK::State& getState() const;	// Get read-only State
	SimTK::State& updState();				// Get writable State

	void setStateDefault(const SimTK::MultibodySystem&);	// Set State from system default State

private:
	SimTK::State	state;	// Particle state
	double			logw;	// Particle state weight
};


// Class for particle vectors with usefull member functions to manage their weight and state and vector size:

class ParticleList {
public:
	ParticleList();
	ParticleList(std::size_t particle_number);
	ParticleList(std::vector <ParticleDynState>& ParticleVector);

	const ParticleDynState getParticle(std::size_t) const;	// Return a read-only particle
	const ParticleDynState operator () (std::size_t) const;	// Operator equal to getParticle

	ParticleDynState& updParticle(std::size_t);			// Return a writable particle
	ParticleDynState& operator [] (std::size_t);		// Operator equal to updParticle

	void addParticle(const ParticleDynState&);	// Add particle to ParticleList
	void operator << (const ParticleDynState&);	// Operator equal to addParticle

	void deleteParticle();							// Delete particle in the last position
	void deleteParticle(std::size_t); 				// Delete particle in specific position
	void deleteParticle(std::size_t, std::size_t); 	// Delete particles between two positions

	const std::vector<ParticleDynState>& getAllParticles() const;	// Return the whole particle list as a read-only vector
	std::vector<ParticleDynState>& updAllParticles();				// Return the whole particle list as a writable vector

	void advanceStates(SimTK::TimeStepper& ts, const double dt);	// Evolve all particles State
	
	void setEqualWeights();			// Set equal logarithmic weights to all particles
	void setEqualLinearWeights();	// Set equal linear weights that sum 1 to all particles
	
	void normalizeWeights();		// Normalize particles' logarithmic weights
	void normalizeLinearWeights();	// Make the particles' weight distribution a probability distribution function (Sum must be 1):

	void calculateESS();			// Calculate the Effective Sample Size (ESS) of the particle list
	const double getESS() const;	// Return the previously calculated ESS 

	void resample();	// Replace particles according to their weights

private:
	std::vector<ParticleDynState> particles;
	double ESS;
};

#endif

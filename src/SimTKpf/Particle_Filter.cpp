#include "Simbody.h"
#include "SimTKpf/PF_Options.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/Particle_Filter.h"
#include "SimTKpf/PF_utilities.h"
#include "SimTKpf/Simbody_Instrument.h"

// ParticleFilter Implementation

ParticleFilter::ParticleFilter(std::size_t particle_number, PF_Options& Options) : 
	Filter_Options(Options), 
	Particles(ParticleList(particle_number)) {}

void ParticleFilter::setOptions(PF_Options newOptions)	{ Filter_Options = newOptions;	}
const PF_Options ParticleFilter::getOptions() const		{ return Filter_Options;		}
PF_Options& ParticleFilter::updOptions()				{ return Filter_Options;		}

void ParticleFilter::setOneOption(PF_Options_index index, double newValue)	{ Filter_Options.setOneOption(index, newValue); }
const double ParticleFilter::getOneOption(PF_Options_index index) const		{ return Filter_Options.getOneOption(index);	}
double& ParticleFilter::updOneOption(PF_Options_index index)				{ return Filter_Options.updOneOption(index);	}

const ParticleList ParticleFilter::getParticleList() const	{ return Particles; }
ParticleList& ParticleFilter::updParticleList()				{ return Particles; }

const std::vector<ParticleDynState> ParticleFilter::getParticleVec() const	{ return Particles.getAllParticles(); }
std::vector<ParticleDynState>& ParticleFilter::updParticleVec()				{ return Particles.updAllParticles(); }

std::size_t ParticleFilter::getParticleNumber() const	{ return Particles.size();		}

void ParticleFilter::setEqualWeights()			{ Particles.setEqualWeights();			}
void ParticleFilter::setEqualLinearWeights()	{ Particles.setEqualLinearWeights();	}

void ParticleFilter::normalizeWeights()			{ Particles.normalizeWeights();			}
void ParticleFilter::normalizeLinearWeights()	{ Particles.normalizeLinearWeights();	}

void ParticleFilter::calculateESS()		{ Particles.calculateESS();		}
double ParticleFilter::getESS() const	{ return Particles.getESS();	}


void ParticleFilter::updateStates(SimTK::TimeStepper& ts, SimTK::Assembler& assembler) {
	
	double const timestep = getOneOption(PF_Options_index::SIMULATION_TIME_STEP);
	double const Motion_StdDev = getOneOption(PF_Options_index::MOTION_STDDEV);
	SimTK::Random::Gaussian motion_noise (0, Motion_StdDev);
	motion_noise.setSeed(getSeed());
	
	Particles.advanceStates(ts, timestep);	// Advance particles state
	
	for (std::size_t i = 0; i < Particles.size(); i++) {	// Add some noise after advancing particles states
		Particles[i].updState().updQ()[0] += motion_noise.getValue();
		assembler.assemble(Particles[i].updState());
	}
}

void ParticleFilter::resample() {
	Particles.resample();
}

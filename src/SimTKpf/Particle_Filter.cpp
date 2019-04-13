#include "Simbody.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/Particle_Filter.h"
#include "SimTKpf/PF_utilities.h"
#include "SimTKpf/Simbody_Instrument.h"

// PF_Options Implementation

PF_Options::PF_Options() {}

PF_Options::PF_Options(	
	const std::size_t PF_PARTICLE_NUMBER,	// Number of particles the filter will work with
	const double PF_SIMULATION_TIME_STEP,	// Time step between two filter iterations
	const double PF_SENSOR_STDDEV,			// Standard deviation of Ground Truth sensor's noise 
	const double PF_SENSOR_STDDEV_MOD,		// Decimal who modifies the sensor standard deviation during updating stage
	const double PF_MOTION_STDDEV,			// Standard deviation of noise added in prediction stage
	const double PF_RESAMPLE_STDDEV			// Standard deviation of noise added after the resampling stage
	) :	PARTICLE_NUMBER(PF_PARTICLE_NUMBER),
		SIMULATION_TIME_STEP(PF_SIMULATION_TIME_STEP),
		SENSOR_STDDEV(PF_SENSOR_STDDEV),
		SENSOR_STDDEV_MOD(PF_SENSOR_STDDEV_MOD),
		MOTION_STDDEV(PF_MOTION_STDDEV),
		RESAMPLE_STDDEV(PF_RESAMPLE_STDDEV) {}

void PF_Options::setOptions( const std::size_t PF_PARTICLE_NUMBER, const double PF_SIMULATION_TIME_STEP, const double PF_SENSOR_STDDEV,
	const double PF_SENSOR_STDDEV_MOD, const double PF_MOTION_STDDEV, const double PF_RESAMPLE_STDDEV){
		PARTICLE_NUMBER = PF_PARTICLE_NUMBER;
		SIMULATION_TIME_STEP = PF_SIMULATION_TIME_STEP;
		SENSOR_STDDEV = PF_SENSOR_STDDEV;
		SENSOR_STDDEV_MOD = PF_SENSOR_STDDEV_MOD;
		MOTION_STDDEV = PF_MOTION_STDDEV;
		RESAMPLE_STDDEV = PF_RESAMPLE_STDDEV;
}

void PF_Options::setOption(PF_Options_index index, std::size_t newValue) {
	if		(index == PF_Options_index::PARTICLE_NUMBER)		PARTICLE_NUMBER			= newValue;
}

void PF_Options::setOption(PF_Options_index index, double newValue) {
	if		(index == PF_Options_index::SIMULATION_TIME_STEP)	SIMULATION_TIME_STEP	= newValue;
	else if (index == PF_Options_index::SENSOR_STDDEV)			SENSOR_STDDEV			= newValue;
	else if (index == PF_Options_index::SENSOR_STDDEV_MOD)		SENSOR_STDDEV_MOD		= newValue;
	else if (index == PF_Options_index::MOTION_STDDEV)			MOTION_STDDEV			= newValue;
	else if (index == PF_Options_index::RESAMPLE_STDDEV)		RESAMPLE_STDDEV			= newValue;
}

const std::size_t PF_Options::getOption(PF_Options_index index) {
	if		(index == PF_Options_index::PARTICLE_NUMBER)		return PARTICLE_NUMBER;
}

const double PF_Options::getOption(PF_Options_index index) {
	if		(index == PF_Options_index::SIMULATION_TIME_STEP)	return SIMULATION_TIME_STEP;
	else if (index == PF_Options_index::SENSOR_STDDEV)			return SENSOR_STDDEV;
	else if (index == PF_Options_index::SENSOR_STDDEV_MOD)		return SENSOR_STDDEV_MOD;
	else if (index == PF_Options_index::MOTION_STDDEV)			return MOTION_STDDEV;
	else if (index == PF_Options_index::RESAMPLE_STDDEV)		return RESAMPLE_STDDEV;
}

std::size_t& PF_Options::updOption(PF_Options_index index) {
	if		(index == PF_Options_index::PARTICLE_NUMBER)		return PARTICLE_NUMBER;
}

double& PF_Options::updOption(PF_Options_index index) {
	if		(index == PF_Options_index::SIMULATION_TIME_STEP)	return SIMULATION_TIME_STEP;
	else if (index == PF_Options_index::SENSOR_STDDEV)			return SENSOR_STDDEV;
	else if (index == PF_Options_index::SENSOR_STDDEV_MOD)		return SENSOR_STDDEV_MOD;
	else if (index == PF_Options_index::MOTION_STDDEV)			return MOTION_STDDEV;
	else if (index == PF_Options_index::RESAMPLE_STDDEV)		return RESAMPLE_STDDEV;
}
// ParticleFilter Implementation

ParticleFilter::ParticleFilter(PF_Options& Options) {
	std::size_t particle_number = Filter_Options.getOption()

	this->Filter_Options = Options;
	Particles = ParticleList();
}

void ParticleFilter::updateStates(ParticleList& particles, SimTK::TimeStepper& ts, SimTK::Assembler& assembler, double timestep, double Motion_StdDev) {
	SimTK::Random::Gaussian motion_noise;
	motion_noise.setSeed(getSeed());
	
	particles.advanceStates(ts, timestep);	// Advance particles state
	
	for (std::size_t i = 0; i < particles.size(); i++) {		// Add some noise after advancing particles states
		particles[i].updState().updQ()[0] += motion_noise.getValue();
		assembler.assemble(particles[i].updState());
	}
}

void ParticleFilter::updateWeights(ParticleList& particles, double GroundTruthInstr_reading, std::vector<Simbody_Instrument> *customInstrVec, double Modified_Instrument_StdDev) {
	double belief;
	Simbody_Instrument *customInstr;

	for (std::size_t i = 0; i < particles.size(); i++){
		customInstr = &customInstrVec->operator[](i);
		customInstr->measure();												// Update instrument reading
		belief = customInstr->read();										// Make the prediction
		particles[i].updWeight() += LogNormalProb(							// Update weights
			belief, GroundTruthInstr_reading, Modified_Instrument_StdDev);	
	}
	particles.normalizeWeights();
}

void ParticleFilter::resample(ParticleList& particles){
	particles.resample();
}

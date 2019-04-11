#include "Simbody.h"
#include "SimTKpf/Particle_Classes.h"

// ParticleDynState Implementation

ParticleDynState::ParticleDynState() : logw(0) {}

ParticleDynState::ParticleDynState(const SimTK::State& stateset, double weight = 0)
	: state(stateset), logw(weight) {}

void ParticleDynState::operator =(const ParticleDynState& p2) {
	logw = p2.getWeight();
	state = p2.getState();
}

bool ParticleDynState::operator <(const ParticleDynState& p2) {
	return logw < p2.getWeight();
}

void ParticleDynState::setWeight(const double& weight)			{ logw = weight;	}
const double ParticleDynState::getWeight() const				{ return logw;		}
double& ParticleDynState::updWeight()							{ return logw;		}

void ParticleDynState::setState(const SimTK::State& stateset)	{ state = stateset;	}
const SimTK::State& ParticleDynState::getState() const			{ return state;		}
SimTK::State& ParticleDynState::updState()						{ return state;		}

void ParticleDynState::setStateDefault(const SimTK::MultibodySystem& system) {
	state = system.getDefaultState();
}


// ParticleList Implementation

ParticleList::ParticleList() {}

ParticleList::ParticleList(std::size_t particle_number) {
	particles.resize(particle_number);
}
ParticleList::ParticleList(std::vector <ParticleDynState>& ParticleVector) {
	particles = ParticleVector;
}

const ParticleDynState ParticleList::getParticle(std::size_t n) const	{ return particles[n]; }
const ParticleDynState ParticleList::operator () (std::size_t n) const	{ return particles[n]; }

ParticleDynState& ParticleList::updParticle(std::size_t n)			{ return particles[n]; }
ParticleDynState& ParticleList::operator [] (std::size_t n)			{ return particles[n]; }

void ParticleList::addParticle(const ParticleDynState& pt) {
	particles.emplace_back(pt);
}
void ParticleList::operator << (const ParticleDynState& pt) {
	particles.emplace_back(pt);
}

void ParticleList::deleteParticle() {
	particles.pop_back();
}
void ParticleList::deleteParticle(std::size_t index) {	
	particles.erase(particles.begin() + index);
}
void ParticleList::deleteParticle(std::size_t first_index, std::size_t last_index) {
	particles.erase(particles.begin() + first_index, particles.begin() + last_index);
}

const std::vector <ParticleDynState>& ParticleList::getAllParticles() const {
	return particles;
}
std::vector <ParticleDynState>& ParticleList::updAllParticles() {
	return particles;
}

void ParticleList::advanceStates(SimTK::TimeStepper& ts, const double dt) {
	for (auto it = particles.begin(); it != particles.end(); ++it) {
		ts.initialize(it->getState());
		ts.stepTo(ts.getTime() + dt);
		it->setState(ts.getState());
	}
}

void ParticleList::setEqualWeights() {
	for (auto it = particles.begin(); it != particles.end(); ++it)
		it->setWeight(0);
}
void ParticleList::setEqualLinearWeights() {
	const double newWeight = std::log(1. / particles.size());

	for (auto it = particles.begin(); it != particles.end(); ++it)
		it->setWeight(newWeight);
}

void ParticleList::normalizeWeights() {
	SimTK::Real maxLogW = particles[0].getWeight();

	for (auto it = particles.begin(); it != particles.end(); ++it)
		maxLogW = std::max<SimTK::Real>(maxLogW, it->getWeight());	// Find the maximum logarithmic weight

	for (auto it = particles.begin(); it != particles.end(); ++it)	// Now normalize weights
		it->updWeight() -= maxLogW;
}
void ParticleList::normalizeLinearWeights() {
	double sumLinearWs = 0;

	for (auto it = particles.begin(); it != particles.end(); ++it)
		sumLinearWs += std::exp(it->getWeight());

	for (auto it = particles.begin(); it != particles.end(); ++it)
		it->updWeight() -= std::log(sumLinearWs);
}

void ParticleList::calculateESS() {
	double sumLinearWeights = 0, LinearW, accum = 0;

	for (auto it = particles.begin(); it != particles.end(); it++) {
		LinearW = std::exp(it->getWeight());
		sumLinearWeights += LinearW;
		accum += LinearW * LinearW;
	}

	if (accum == 0)	ESS = 0;	// ESS == 0 when all linear weights are zero
	else			ESS = (sumLinearWeights*sumLinearWeights) / (particles.size() * accum);	// ESS == 1 when equal weights
}

const double ParticleList::getESS() const { return ESS; }

void ParticleList::resample() {

	std::size_t i, j, particle_number = particles.size();
	std::vector <ParticleDynState> ResamplingPtcls;
	std::vector <double> LinearWeights, accumLinearWs, equalWeights;
	double sumLinearWeights = 0, value = 0, maxLogW = 0;

	// Get maximum logarithmic weight:
	
	for (auto it = particles.begin(); it != particles.end(); ++it)
		if (maxLogW < it->getWeight()) maxLogW = it->getWeight();
	
	// Get particles linear weights, calculate their sum and normalize them:

	for (auto it = particles.begin(); it != particles.end(); ++it) {
		value = std::exp(it->getWeight() - maxLogW);	// value here contains linear weight
		LinearWeights.push_back(value);
		sumLinearWeights += value;
	}

	for (i = 0; i < particle_number; ++i)
		LinearWeights[i] /= sumLinearWeights;

	// Arrange a vector with accumulative linear weights:

	value = 0;
	for (auto it = LinearWeights.begin(); it != LinearWeights.end(); ++it) {
		value += *it;		// value here contains accumulative normalized linear weights
		accumLinearWs.push_back(value);
	}
	accumLinearWs[particle_number - 1] = 1.1;

	// Arrange a vector with accumulative equal weights:

	i = 1;
	value = 0.99999 / (particle_number - 1);	// value here contains equal weights step
	for (auto it = LinearWeights.begin(); it != LinearWeights.end() - 1; ++it, ++i) {
		equalWeights.push_back(value*i);
	}
	equalWeights.push_back(1.0);

	// Select particles to later replacement:

	i = 0, j = 0;
	while (i < particle_number) {
		if (equalWeights[i] <= accumLinearWs[j]) {
			ResamplingPtcls.emplace_back(particles[j]);
			i++;
		}
		else
			j++;
	}

	// Replace particles:
	particles = ResamplingPtcls;

	// Make all particles weights equal
	setEqualWeights();
}

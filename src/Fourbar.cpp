//#include <cstdio>		// Already included in "Simbody.h"?
//#include <array>		// Already included in "Simbody.h"?
//#include <algorithm>	// Maybe useful in future
#include "Simbody.h"
#include "Gyroscope.h"

// Return a integer to be used as a seed in random functions. Said integer contains the seconds elapsed since 1/1/1970. 
int getSeed() {
	timespec ts;
	long long nanoseconds;
	double seconds;
	
	clock_gettime(CLOCK_REALTIME, &ts);
	nanoseconds = SimTK::timespecToNs(ts);
	seconds = SimTK::nsToSec(nanoseconds);

	return int(seconds);
}
// We are gonna define a new class which the PF particles will belong to:

class ParticleDynState {
public:
	ParticleDynState() : logw(0) {}			// Default constructor. State keeps empty

	ParticleDynState(const SimTK::State& stateset, double weight=0)
	: state(stateset), logw(weight) {}		// Constructor given a state. Weight is zero if unspecified
	
	void operator =(const ParticleDynState& p2) {	// Operator to copy particles
		logw = p2.getWeight();
		state = p2.getState();
	}

	bool operator <(const ParticleDynState& p2) {	// Operator necessary to sort particles
		return logw < p2.getWeight();
	}

	void setWeight(const double& weight)	{ logw = weight; }			// Set weight value
	const double getWeight() const			{ return logw;	 }			// Get read-only weight value
	double& updWeight()						{ return logw;	 }			// Get writable weight value

	void setState(const SimTK::State& stateset)	{ state = stateset;	}	// Set State
	const SimTK::State& getState() const		{ return state;		}	// Get read-only State
	SimTK::State& updState()					{ return state;		}	// Get writable State

	void setStateDefault(const SimTK::MultibodySystem& system)			// Set State from system default State
		{ state = system.getDefaultState(); }

private:
	SimTK::State	state;	// Particle state
	double			logw;	// Particle state weight
};

// Class for particle vectors with usefull member functions to manage their weight and state:

class ParticleList {
public:
	ParticleList() {}
	ParticleList(std::size_t particle_number) {
		for (std::size_t i = 0; i < particle_number; ++i)
			particles.push_back(ParticleDynState());
	}
	ParticleList(std::vector <ParticleDynState>& ParticleVector) {
		particles = ParticleVector;
	}	
	
	const ParticleDynState getParticle(std::size_t n)	{ return particles[n]; }	// Return a read-only particle
	const ParticleDynState operator () (std::size_t n)	{ return particles[n]; }	// Operator equal to getParticle

	ParticleDynState& updParticle(std::size_t n)		{ return particles[n]; }	// Return a writable particle
	ParticleDynState& operator [] (std::size_t n)		{ return particles[n]; }	// Operator equal to updParticle

	void addParticle(const ParticleDynState& pt) {	// Add particle to ParticleList
		particles.push_back(pt);
	}
	void operator << (const ParticleDynState& pt) {	// Operator equal to addParticle
		particles.push_back(pt);
	}

	void deleteParticle() {		// Delete particle in the last position
		particles.pop_back();
	}
	void deleteParticle(std::size_t index) {	// Delete particle in specific position
		particles.erase(particles.begin() + index);
	}
	void deleteParticle(std::size_t first_index, std::size_t last_index) {	// Delete particles between two positions
		particles.erase(particles.begin() + first_index, particles.begin() + last_index);
	}

	std::vector <ParticleDynState> getAllParticles() {	// Return the whole particle list as a vector
		return particles;
	}
	
	void advanceStates(SimTK::TimeStepper& ts, const double dt) {	// Evolve all particles State
		for (auto it = particles.begin(); it != particles.end(); ++it) {
			ts.initialize(it->getState());
			ts.stepTo(ts.getTime() + dt);
			it->setState(ts.getState());
		}
	}
	
	void setEqualWeights() {	// Set equal logarithmic weights to all particles
		for (auto it = particles.begin(); it != particles.end(); ++it)
			it->setWeight(0);		
	}

	void setEqualLinearWeights() {	// Set equal linear weights that sum 1 to all particles
		const double newWeight = std::log(1. / particles.size());

		for (auto it = particles.begin(); it != particles.end(); ++it)
			it->setWeight(newWeight);
	}

	void normalizeWeights() {	// Normalize particles' logarithmic weights
		SimTK::Real maxLogW = particles[0].getWeight();

		for (auto it = particles.begin(); it != particles.end(); ++it)
			maxLogW = std::max<SimTK::Real>(maxLogW, it->getWeight());	// Find the maximum logarithmic weight

		for (auto it = particles.begin(); it != particles.end(); ++it)	// Now normalize weights
			it->updWeight() -= maxLogW;
	}

	void normalizeLinearWeights() {	// Make the particles' weight distribution a probability distribution function (Sum must be 1):
		double sumLinearWs = 0;

		for (auto it = particles.begin(); it != particles.end(); ++it)
			sumLinearWs += std::exp(it->getWeight());
		
		for (auto it = particles.begin(); it != particles.end(); ++it)
			it->updWeight() -= std::log(sumLinearWs);
	}

	void calculateESS() {	// Calculate the Effective Sample Size (ESS) of the particle list
		double sumLinearWeights = 0, LinearW, accum = 0;
		
		for (auto it = particles.begin(); it != particles.end(); it++){
			LinearW = std::exp(it->getWeight());
			sumLinearWeights += LinearW;
			accum += LinearW * LinearW;
		}

		if (accum == 0)	ESS = 0;	// ESS == 0 when all linear weights are zero
		else			ESS = (sumLinearWeights*sumLinearWeights) / (particles.size() * accum);	// ESS == 1 when equal weights
	}

	const double getESS() { return ESS; }

	void resample() {	// More efficient?
		std::size_t i, j, particle_number = particles.size();
		std::vector <ParticleDynState> ResamplingPtcls;
		std::vector <double> LinearWeights, accumLinearWs, equalWeights;
		double sumLinearWeights = 0, value, maxLogW;

		// Sort particles and get maximum logarithmic weight
		std::sort(particles.begin(), particles.end());
		maxLogW = particles[particle_number - 1].getWeight();
		
		// Get particles linear weights, calculate their sum and normalize them
		for (auto it = particles.begin(); it != particles.end(); ++it) {
			value = std::exp(it->getWeight() - maxLogW);	// value here contains linear weight
			LinearWeights.push_back(value);
			sumLinearWeights += value;
		}
		
		for (i = 0; i < particle_number; ++i)
			LinearWeights[i] /= sumLinearWeights;
		
		// Arrange a vector with accumulative linear weights
		value = 0;
		for (auto it = LinearWeights.begin(); it != LinearWeights.end(); ++it){
			value += *it;		// value here contains accumulative normalized linear weights
			accumLinearWs.push_back(value);
		}
		accumLinearWs[particle_number - 1] = 1.1;

		// Arrange a vector with accumulative equal weights
		i = 1;
		value = 0.99999 / (particle_number - 1);	// value here contains equal weights step
		for (auto it = LinearWeights.begin(); it != LinearWeights.end() - 1; ++it, ++i) {
			equalWeights.push_back(value*i);
		}
		equalWeights.push_back(1.0);

		// Select particles to later replacement
		i = 0, j = 0;
		while(i < particle_number){
			if (equalWeights[i] <= accumLinearWs[j]) {
				ResamplingPtcls.push_back(particles[j]);
				i++;
			}
			else
				j++;
		}

		// Replace particles
		particles = ResamplingPtcls;
	}

private:
	std::vector<ParticleDynState> particles;
	double ESS;
};

// Function to make a single State progress.
void advance(SimTK::State& state, SimTK::TimeStepper& ts , const double dt) {		
	ts.initialize(state);
	ts.stepTo(ts.getTime() + dt);
	state = ts.getState();

}

// Return the exponent of a gaussian distribution function evaluated at x, given the mean and standart deviation s: 
double NormalProb(const double x, const double mean, const double s) {
	double y = ((mean - x) / s);
	return -0.5*y*y;
}

// Update each gyroscope reading, and return their stardard deviation
double getGyrsStdDev(std::vector <Gyroscope>& gyrs) {
	double sum = 0;
	double mean = 0;
	double difference;
	std::size_t size = gyrs.size();

	for (std::size_t i = 0; i < size; i++) {
		gyrs[i].measure();
		sum += gyrs[i].read();
	}

	mean = sum / size;
	sum = 0;

	for (std::size_t i = 0; i < size; i++) {
		difference = gyrs[i].read() - mean;
		sum += difference * difference;
	}
	
	return sqrt(sum / (size - 1));
}

int main() {
	std::array <double, 4> BAR_LENGHTS = { // Examples: Double cranck: (2, 4, 3, 4), Crank-Rocker: (4, 2, 3, 4)
		2,		// Bar1 lenght
		4,		// Bar2 lenght
		3,		// Bar3 lenght
		4 };	// Bar4 lenght
	const double BAR_WIDTH = 0.05;
	const double SIM_TIME_STEP = 0.006;
	const double SIMULATION_TIME = 60;
	const double GYROSCOPE_STDDEV = 0.01;
	const double FILTER_STDDEV = 0.02;
	std::size_t PARTICLE_NUMBER = 200;

	/*
	const SimTK::String WINDOW_TITLE = "Four bar linkage // Particle Filter";
	const double TIME_SCALE = 1;
	const double FRAME_RATE = 30;
	const double VIZ_REPORT_INTERVAL = TIME_SCALE / FRAME_RATE;	// Overwrite to specific interval report	*/
	

	try {
		// Create the system.
		SimTK::MultibodySystem system;
		system.setUseUniformBackground(true);
		SimTK::SimbodyMatterSubsystem matter(system);
		SimTK::GeneralForceSubsystem forces(system);
		SimTK::Force::UniformGravity gravity(forces, matter, SimTK::Vec3(0, -9.8, 0));

		SimTK::Body::Rigid Body1, Body2, Body3;
		
		matter.Ground().addBodyDecoration(SimTK::Transform(SimTK::Rotation(SimTK::Pi/2, SimTK::Vec3(0 , 0, 1)), 
			SimTK::Vec3(-BAR_LENGHTS[0] / 2, 0, 0)), SimTK::DecorativeCylinder(BAR_WIDTH, BAR_LENGHTS[0] / 2));

		Body1.setDefaultRigidBodyMassProperties(SimTK::MassProperties(1.0, SimTK::Vec3(0), SimTK::Inertia(1)));
		Body1.addDecoration(SimTK::Transform(SimTK::Vec3(0, BAR_LENGHTS[1] / 2, 0)),
			SimTK::DecorativeCylinder(BAR_WIDTH, BAR_LENGHTS[1] / 2));
		SimTK::MobilizedBody::Pin Bar1(matter.Ground(), SimTK::Transform(SimTK::Vec3(-BAR_LENGHTS[0], 0, 0)),
			Body1, SimTK::Transform(SimTK::Vec3(0, BAR_LENGHTS[1], 0)));

		Body2.setDefaultRigidBodyMassProperties(SimTK::MassProperties(1.0, SimTK::Vec3(0), SimTK::Inertia(1)));
		Body2.addDecoration(SimTK::Transform(SimTK::Vec3(0, BAR_LENGHTS[2] / 2, 0)),
			SimTK::DecorativeCylinder(BAR_WIDTH, BAR_LENGHTS[2] / 2));
		SimTK::MobilizedBody::Pin Bar2(Bar1, SimTK::Transform(SimTK::Vec3(0)),
			Body2, SimTK::Transform(SimTK::Vec3(0, BAR_LENGHTS[2], 0)));

		Body3.setDefaultRigidBodyMassProperties(SimTK::MassProperties(1.0, SimTK::Vec3(0), SimTK::Inertia(1)));
		Body3.addDecoration(SimTK::Transform(SimTK::Vec3(0, BAR_LENGHTS[3] / 2, 0)),
			SimTK::DecorativeCylinder(BAR_WIDTH, BAR_LENGHTS[3] / 2));
		SimTK::MobilizedBody::Pin Bar3(Bar2, SimTK::Transform(SimTK::Vec3(0)),
			Body3, SimTK::Transform(SimTK::Vec3(0, BAR_LENGHTS[3], 0)));
		
		SimTK::Constraint::Ball(matter.Ground(), SimTK::Vec3(0), Bar3, SimTK::Vec3(0));
		//SimTK::Constraint::CoincidentPoints(matter.Ground(), SimTK::Vec3(0), Bar3, SimTK::Vec3(0));	// Same as Ball
		
		/*		// Set up visualization.
		SimTK::Visualizer viz(system);
		system.addEventReporter(new SimTK::Visualizer::Reporter(viz,VIZ_REPORT_INTERVAL ));
		viz.setWindowTitle(WINDOW_TITLE);
		viz.setDesiredFrameRate(FRAME_RATE);	//*/
		
		// Initialize the system, reference state and particles.
		system.realizeTopology();
		SimTK::State RefState = system.getDefaultState();	// Reference State
		ParticleList particles(PARTICLE_NUMBER);			// Particle vector

		// Create and add assembler.
		SimTK::Assembler assembler(system);
		assembler.setSystemConstraintsWeight(1);

		// We will random assign the reference state and particle states in the next block
		{
			SimTK::Random::Uniform randomAngle(0, 2 * SimTK::Pi);
			randomAngle.setSeed(getSeed());

			std::cout << "Assigning and assembling Reference state...";
			RefState.updQ()[0] = randomAngle.getValue();
			assembler.assemble(RefState);

			std::cout << " Done.\nAssigning and assembling Particle states...";

			for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {	// Particle vector random assigment
				particles[i].setStateDefault(system);
				particles[i].updState().updQ()[0] = randomAngle.getValue();
				assembler.assemble(particles[i].updState());
			}
			std::cout << " Done.\n" << std::endl;
		}
		particles.setEqualWeights();

		// Simulate it.
		SimTK::RungeKuttaMersonIntegrator integ(system);
		SimTK::TimeStepper ts(system, integ);
		
		Gyroscope gyr(system, matter, RefState, BAR_LENGHTS[0], GYROSCOPE_STDDEV);	// Gyroscope used by reference state
		std::vector<Gyroscope> pargyr;									// Vector of gyroscopes used by particles
		for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)				// Arrange gyroscope vector
			pargyr.push_back(Gyroscope(system, matter, particles[i].updState(), BAR_LENGHTS[0], GYROSCOPE_STDDEV));
		Gyroscope::setGlobalSeed(getSeed());

		double CPUtimestart;	// Reference to check CPU time
		double bel;				// Advanced angular velocity (belief)
		
		SimTK::Random::Gaussian noise(0, FILTER_STDDEV);	// Gaussian noise for later use
		noise.setSeed(getSeed());
		CPUtimestart = SimTK::cpuTime();

		for (double time = 0; time <= SIMULATION_TIME; time += SIM_TIME_STEP) {	// Loop to slowly advance simulation

			std::cout << "\n NEXT ITERATION...\n\nCurrent real time: " << time << " s\nCurrent CPU time: " 
				<< SimTK::cpuTime() - CPUtimestart << " s" << std::endl;

			advance(RefState, ts, SIM_TIME_STEP);			// Advance reference state
			particles.advanceStates(ts, SIM_TIME_STEP);		// Advance particles
			std::cout << "\nTime advanced " << SIM_TIME_STEP << " s" << std::endl;

			gyr.measure();					// Update reference state gyroscope reading
			
			for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)	// Update particles gyroscopes reading
				pargyr[i].measure();
			

			for (std::size_t i = 0; i < PARTICLE_NUMBER; i++){
				bel = pargyr[i].read();
				particles[i].updWeight() += NormalProb(bel, gyr.read(), GYROSCOPE_STDDEV);	// Update weights
			}
			particles.normalizeWeights();

			// Add some noise post-predictions	// AFTER PREDICTION?
			for (std::size_t i = 0; i < PARTICLE_NUMBER; i++){
				particles[i].updState().updQ()[0] += noise.getValue();
				assembler.assemble(particles[i].updState());
			}

			printf("\nPARTICLE SUMMARY:\n");
			for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)
				printf("\nParticle %3.d \tOmega =%9.5f\tLog(w) = %10.5f\tAngle = %7.3f",
					i + 1, pargyr[i].read(), particles[i].getWeight(), particles[i].getState().getQ()[0]);

			std::cout << "\n\nReference Gyroscope: " << gyr.read() << std::endl;
			std::cout << "Reference Angle: " << RefState.getQ()[0] << std::endl;


			particles.calculateESS();
			printf("\n ESS = %f %%\n", particles.getESS() * 100);
			
			if (particles.getESS() < 0.5) {		// Resample when < 50 % of effective particles
				
				std::cout << "\nResample is necessary. Resampling... ";
				particles.resample();
				std::cout << "Resample done!" << std::endl;
			}
			
			std::cout << "\nPress Enter to keep iterating" << std::endl;
			getchar();
		}
		std::cout << "\nSIMULATION ENDED. Press Enter to close this window." << std::endl;
		getchar();
		
	}
	catch (const std::exception& e) {
		std::cout << "Error: " << e.what() << std::endl;
		getchar();
		return 1;
	}
}
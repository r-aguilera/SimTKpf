//#include <cstdio>		// Already included in "Simbody.h"?
//#include <array>		// Already included in "Simbody.h"?
//#include <cmath>		// Already included in "Simbody.h"?
//#include <algorithm>	// Maybe useful in future
#include "Simbody.h"
#include "Gyroscope.h"
#include "PF_utilities.h"

// Return a integer to be used as a seed in random functions. Said integer contains the seconds elapsed since 1/1/1970. 
inline int getSeed() {
	timespec ts;
	long long nanoseconds;
	double seconds;
	
	clock_gettime(CLOCK_REALTIME, &ts);
	nanoseconds = SimTK::timespecToNs(ts);
	seconds = SimTK::nsToSec(nanoseconds);

	return int(seconds);
}

// Return the given angle, expressed in the range [0, 2*Pi). Input/Output in radians!
inline double to2Pi(double Angle){
	
	bool isNegative = Angle < 0;
	double Output_Angle = fmod(Angle, 2 * SimTK::Pi);
	if (isNegative) Output_Angle += 2 * SimTK::Pi;

	return Output_Angle;
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
	const std::size_t PARTICLE_NUMBER = 200;

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
		std::vector<Gyroscope> pargyr;												// Vector of gyroscopes used by particles
		pargyr.reserve(PARTICLE_NUMBER);
		for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)							// Arrange gyroscope vector
			pargyr.push_back(Gyroscope(system, matter, particles[i].updState(),
				BAR_LENGHTS[0], GYROSCOPE_STDDEV));
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
					i + 1, pargyr[i].read(), particles[i].getWeight(), to2Pi(particles[i].getState().getQ()[0]));
			
			std::cout << "\n\nReference Gyroscope: " << gyr.read() << std::endl;
			std::cout << "Reference Angle: " << to2Pi(RefState.getQ()[0]) << std::endl;


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
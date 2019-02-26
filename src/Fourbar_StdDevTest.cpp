//#include <cstdio>		// Already included in "Simbody.h"?
//#include <array>		// Already included in "Simbody.h"?
//#include <algorithm>	// Maybe useful in future
#include <fstream>
#include <string>
#include <conio.h>
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
inline double to2Pi(double Angle) {

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
	const double GYROSCOPE_STDDEV = 0.01;
	double FILTER_STDDEV;
	const std::size_t PARTICLE_NUMBER = 200;
	const double SIMULATION_TIME = 0.252;		// 42 timesteps

	/*
	const SimTK::String WINDOW_TITLE = "Four bar linkage // Particle Filter";
	const double TIME_SCALE = 1;
	const double FRAME_RATE = 30;
	const double VIZ_REPORT_INTERVAL = TIME_SCALE / FRAME_RATE;	// Overwrite to specific interval report	*/
	
	do{
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
				
			// Initialize the system, reference state and particles.
			system.realizeTopology();
			SimTK::State RefState = system.getDefaultState();	// Reference State
			ParticleList particles(PARTICLE_NUMBER);			// Particle vector

			// Create and add assembler.
			SimTK::Assembler assembler(system);
			assembler.setSystemConstraintsWeight(1);

			// Define test variables
			std::fstream File;
			SimTK::String filename = "Fourbar_StdDevTest.txt";	// This file will contain results when test finishes
			SimTK::String savename = "Savestate.txt";			// This file will contain variables data in case test get interrupted
			std::array<double, 10> deviations = {	// FILTER_STDDEV values to test
				0.001,	0.002,	0.005,
				0.01,	0.02,	0.05,
				0.1,	0.2,	0.5,
				1
			};
			std::size_t tries = 100;	// Number of convergence tries in a simulation
			int convs = 0;				// Number of convergences in a simulation
			double CPUtimestart;		// Reference to check CPU time
			double CPUtotaltime = 0;

			// Check if test is in progress
			int saved_try= 0;

			File.open(savename, std::ios::in);
			if (File) {
				SimTK::String fileline;

				std::getline(File, fileline);
				std::stringstream(fileline) >> saved_try;

				std::getline(File, fileline);
				std::stringstream(fileline) >> convs;
			
				std::getline(File, fileline);
				std::stringstream(fileline) >> CPUtotaltime;
			}
			CPUtimestart = SimTK::cpuTime();

			std::cout << "Press Escape to stop test\n" << std::endl;

			for (std::size_t StdDev_i = int(floor(saved_try/tries)); StdDev_i < deviations.size(); ++StdDev_i) {

				FILTER_STDDEV = deviations[StdDev_i];
				std::cout << "Starting test with FILTER_STDDEV = " << FILTER_STDDEV << std::endl;


				for (std::size_t try_n = fmod(saved_try, tries); try_n < tries; ++try_n) {

					// We will random assign the reference state and particle states in the next block
					{
						SimTK::Random::Uniform randomAngle(0, 2 * SimTK::Pi);
						randomAngle.setSeed(getSeed());

						RefState.updQ()[0] = randomAngle.getValue();
						assembler.assemble(RefState);

						for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {	// Particle vector random assigment
							particles[i].setStateDefault(system);
							particles[i].updState().updQ()[0] = randomAngle.getValue();
							assembler.assemble(particles[i].updState());
						}
					}
					particles.setEqualWeights();

					// Simulate it.
					SimTK::RungeKuttaMersonIntegrator integ(system);
					SimTK::TimeStepper ts(system, integ);
		
					Gyroscope gyr(system, matter, RefState, BAR_LENGHTS[0], GYROSCOPE_STDDEV);	// Gyroscope used by reference state
					std::vector<Gyroscope> pargyr;												// Vector of gyroscopes used by particles
					for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)							// Arrange gyroscope vector
						pargyr.push_back(Gyroscope(system, matter, particles[i].updState(),
							BAR_LENGHTS[0], GYROSCOPE_STDDEV));
					Gyroscope::setGlobalSeed(getSeed());

					double bel;				// Advanced angular velocity (belief)
		
					SimTK::Random::Gaussian noise(0, FILTER_STDDEV);	// Gaussian noise for later use
					noise.setSeed(getSeed());
					CPUtimestart = SimTK::cpuTime();

					for (double time = 0; time <= SIMULATION_TIME; time += SIM_TIME_STEP) {	// Loop to slowly advance simulation

						if (_kbhit()) {
							char key = _getch();
							if (key == 27) return 0;
						}

						advance(RefState, ts, SIM_TIME_STEP);			// Advance reference state
						particles.advanceStates(ts, SIM_TIME_STEP);		// Advance particles

						gyr.measure();					// Update reference state gyroscope reading

						for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)	// Update particles gyroscopes reading
							pargyr[i].measure();


						for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {
							bel = pargyr[i].read();
							particles[i].updWeight() += NormalProb(bel, gyr.read(), GYROSCOPE_STDDEV);	// Update weights
						}
						particles.normalizeWeights();

						// Add some noise post-predictions	// AFTER PREDICTION?
						for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {
							particles[i].updState().updQ()[0] += noise.getValue();
							assembler.assemble(particles[i].updState());
						}

						particles.calculateESS();

						if (particles.getESS() < 0.5) {		// Resample when < 50 % of effective particles

							particles.resample();
						}

						// Here ends one iteration
					}
				
					//Here ends one simulation try

					// Convergence criteria:
					double Ref_Angle, Estimated_Angle = 0;

					Ref_Angle = to2Pi(RefState.getQ()[0]); // Get reference State wanted angle

					particles.setEqualLinearWeights();

					for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {

						double partAngle = to2Pi(particles[i].getState().getQ()[0]);	// Get particle State wanted angle
						double partLinW = exp(particles[i].getWeight());				// Get particle linear weight

						Estimated_Angle += partAngle * partLinW;
					}

					if (abs(Ref_Angle - Estimated_Angle) <= 5 * SimTK::Pi / 180 ||
						abs(Ref_Angle - Estimated_Angle) >= 355 * SimTK::Pi / 180)
						++convs;
				
					CPUtotaltime += SimTK::cpuTime() - CPUtimestart;
					saved_try++;

					File.open(savename, std::ios::out);
					File << saved_try << std::endl;
					File << convs << std::endl;
					File << CPUtotaltime << std::endl;
					File.close();
				
					CPUtimestart = SimTK::cpuTime();
				}
			
				//Here ends one StdDev value test
			
				std::cout << "\n\tAchieved test in " << CPUtotaltime << " s." << std::endl;

				File.open(filename, std::ios::app);
				File << "StdDev: " << FILTER_STDDEV << "\t " << "\tTest time: " << CPUtotaltime << " s"
					<< "\t" << convs << "/"<< tries <<" success" << std::endl;
				File.close();

				convs = 0;
				CPUtotaltime = 0;
				CPUtimestart = SimTK::cpuTime();

				File.open(savename, std::ios::out);
				File << saved_try << std::endl;
				File << convs << std::endl;
				File << CPUtotaltime << std::endl;
				File.close();

			}

			// Here ends all tests

			std::cout << "\nSIMULATION ENDED. Press Enter to close this window." << std::endl;
			getchar();
			return 0;
		
		}
		catch (const std::exception& e) {
			std::cout << "\nError: " << e.what() << std::endl;
		}
	} while (true);
}
//#include <cstdio>		// Already included in "Simbody.h"
//#include <array>		// Already included in "Simbody.h"
//#include <cmath>		// Already included in "Simbody.h"
//#include <algorithm>	// Maybe useful in future
#include <fstream>
#include <string>
//#include <conio.h>
#include "Simbody.h"
#include "SimTKpf.h"
#include "Fourbar.h"

int main() {
	const double SIMULATION_TIME = 0.252;		// 42 timesteps
	double SIM_TIME_STEP = 0.006;
	double GYROSCOPE_STDDEV = 0.01;
	double UPDATING_STDDEV_MOD = 10;
	double MOTION_STDDEV = 0;
	double RESAMPLE_STDDEV = 0.02;
	PF_Options PARTICLE_FILTER_OPTIONS(
		SIM_TIME_STEP,
		GYROSCOPE_STDDEV,
		UPDATING_STDDEV_MOD,
		MOTION_STDDEV,
		RESAMPLE_STDDEV
	);
	std::size_t PARTICLE_NUMBER = 200;
	ParticleFilter FILTER(PARTICLE_NUMBER, PARTICLE_FILTER_OPTIONS);

	std::vector <double> BAR_LENGHTS = { // Examples: Double Crank: (2, 4, 3, 4), Crank-Rocker: (4, 2, 3, 4)
		2,		// Bar1 lenght
		4,		// Bar2 lenght
		3,		// Bar3 lenght
		4 };	// Bar4 lenght
	GrashofCondition FOURBAR_CONFIGURATION = evaluateGrashof(BAR_LENGHTS);

	// do{
		try {
			// Create the system.
			SimTK::MultibodySystem system;
			SimTK::SimbodyMatterSubsystem matter(system);
			SimTK::GeneralForceSubsystem forces(system);
			SimTK::Force::UniformGravity gravity(forces, matter, SimTK::Vec3(0, -9.8, 0));

			SimTK::Body::Rigid Body1, Body2, Body3;

			Body1.setDefaultRigidBodyMassProperties(SimTK::MassProperties(1.0, SimTK::Vec3(-BAR_LENGHTS[1] / 2, 0, 0), SimTK::Inertia(1)));
			SimTK::MobilizedBody::Pin Bar1(matter.Ground(), SimTK::Transform(SimTK::Vec3(-BAR_LENGHTS[0], 0, 0)),
				Body1, SimTK::Transform(-SimTK::Vec3(BAR_LENGHTS[1], 0, 0)));

			Body2.setDefaultRigidBodyMassProperties(SimTK::MassProperties(1.0, SimTK::Vec3(-BAR_LENGHTS[2] / 2, 0, 0), SimTK::Inertia(1)));
			SimTK::MobilizedBody::Pin Bar2(Bar1, SimTK::Transform(SimTK::Vec3(0)),
				Body2, SimTK::Transform(-SimTK::Vec3(BAR_LENGHTS[2], 0, 0)));

			Body3.setDefaultRigidBodyMassProperties(SimTK::MassProperties(1.0, SimTK::Vec3(-BAR_LENGHTS[3] / 2, 0, 0), SimTK::Inertia(1)));
			SimTK::MobilizedBody::Pin Bar3(Bar2, SimTK::Transform(SimTK::Vec3(0)),
				Body3, SimTK::Transform(-SimTK::Vec3(BAR_LENGHTS[3], 0, 0)));

			SimTK::Constraint::Ball(matter.Ground(), SimTK::Vec3(0), Bar3, SimTK::Vec3(0));
				
			// Initialize the system, reference state and particles.
			system.realizeTopology();
			SimTK::State RefState;
			ParticleList particles(PARTICLE_NUMBER);

			// Create and add assembler.
			SimTK::Assembler assembler(system);
			assembler.setSystemConstraintsWeight(1);

			// Define test variables
			std::fstream File;
			SimTK::String filename = "Fourbar_StdDevTest.txt";				// This file will contain results when test finishes
			SimTK::String savename = "Fourbar_StdDevTest_Savestate.txt";	// This file will contain variables data in case test get interrupted
			std::array<double, 10> deviations = {	// MOTION_STDDEV values to test
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

			std::cout << "Press Q to quit test\n" << std::endl;

			for (std::size_t StdDev_i = static_cast<int>(floor(saved_try/tries)); StdDev_i < deviations.size(); ++StdDev_i) {

				MOTION_STDDEV = deviations[StdDev_i];
				FILTER.setOneOption(PF_Options_index::MOTION_STDDEV, MOTION_STDDEV);
				std::cout << "Starting test with MOTION_STDDEV = " << MOTION_STDDEV << std::endl;

				for (std::size_t try_n = static_cast<int>(fmod(saved_try, tries)); try_n < tries; ++try_n) {

					// We will random assign the reference state and particle states
					assemble_Fourbar(FOURBAR_CONFIGURATION, system, assembler, RefState, FILTER);
					FILTER.setEqualWeights();

					// Simulate it.
					SimTK::RungeKuttaMersonIntegrator integ(system);
					SimTK::TimeStepper ts(system, integ);
		
					Gyroscope gyr(system, matter, RefState, BAR_LENGHTS[0], GYROSCOPE_STDDEV);	// Gyroscope used by reference state
					std::vector<Gyroscope> pargyr;												// Vector of gyroscopes used by particles
					
					pargyr.reserve(PARTICLE_NUMBER);
					
					for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {							// Arrange gyroscope vector
						pargyr.push_back(Gyroscope(system, matter, FILTER.updParticleVec()[i].updState(), BAR_LENGHTS[0], 0));
					}
					Gyroscope::setGlobalSeed(getSeed());

					SimTK::Random::Gaussian resample_noise(0, RESAMPLE_STDDEV);	// Gaussian noise added during resampling
					resample_noise.setSeed(getSeed());

					CPUtimestart = SimTK::cpuTime();

					for (double time = 0; time <= SIMULATION_TIME; time += SIM_TIME_STEP) {	// Loop to slowly advance simulation

#if 0
						if (_kbhit()) {
							char key = toupper(_getch());
							if (key == 'Q') return 0;
						}
#endif

						advance(RefState, ts, SIM_TIME_STEP);					// Advance reference state
						FILTER.updateStates(ts, assembler);						// Advance particles state
						gyr.measure();											// Update reference gyroscope reading
						FILTER.updateWeights<Gyroscope>(gyr.read(), pargyr);	// Update particles gyroscope reading and weights
						FILTER.calculateESS();									// Calculate particles ESS
						
						if (FILTER.getESS() < 0.5) {		// Resample when < 50 % of effective particles
							FILTER.resample();
							for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {	// Noise added to angular velocity
								double Rate = Bar1.getRate(FILTER.updParticleVec()[i].updState());
								Bar1.setRate(FILTER.updParticleVec()[i].updState(), Rate + resample_noise.getValue());
							}
						}	
					}	// Here ends one iteration
				
					// Convergence criteria:
					double Ref_Angle, Estimated_Angle = 0;

					Ref_Angle = to2Pi(RefState.getQ()[0]); // Get reference State wanted angle

					FILTER.normalizeLinearWeights();

					for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {

						double partAngle = to2Pi(FILTER.updParticleVec()[i].getState().getQ()[0]);	// Get particle State wanted angle
						double partLinW = exp(FILTER.updParticleVec()[i].getWeight());				// Get particle linear weight

						Estimated_Angle += partAngle * partLinW;
					}
					if (abs(Ref_Angle - Estimated_Angle) <= 5 * SimTK::Pi / 180 ||
						abs(Ref_Angle - Estimated_Angle) >= 355 * SimTK::Pi / 180)
					{
						++convs;
					}

					CPUtotaltime += SimTK::cpuTime() - CPUtimestart;
					saved_try++;

					File.open(savename, std::ios::out);
					File << saved_try << std::endl;
					File << convs << std::endl;
					File << CPUtotaltime << std::endl;
					File.close();
				
					CPUtimestart = SimTK::cpuTime();
				
					}	//Here ends tries loop
			
				std::cout << "\tAchieved test in " << CPUtotaltime << " s.\n" << std::endl;

				File.open(filename, std::ios::app);
				File << "StdDev: " << MOTION_STDDEV << "\t " << "\tTest time: " << CPUtotaltime << " s"
					<< "\t" << convs << "/" << tries << " success" << std::endl;
				File.close();

				convs = 0;
				CPUtotaltime = 0;
				CPUtimestart = SimTK::cpuTime();
				
				}	// Here ends StdDev values loop

			File.open(savename, std::ios::out);
			File << saved_try << std::endl;
			File << convs << std::endl;
			File << CPUtotaltime << std::endl;
			File.close();

			std::cout << "\nSIMULATION ENDED. Press Enter to close this window." << std::endl;
			getchar();
			return 0;

		}	// Here ends all tests

		catch (const std::exception& e) {
			std::cout << "\nError: " << e.what() << std::endl;
		}
		//} while (true);
}

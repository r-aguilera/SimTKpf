//#include <cstdio>		// Already included in "Simbody.h"?
//#include <array>		// Already included in "Simbody.h"?
//#include <cmath>		// Already included in "Simbody.h"?
//#include <algorithm>	// Maybe useful in future
#include "Simbody.h"
#include "SimTKpf.h"
#include "Fourbar/assemble_Fourbar.h"
#include "Fourbar/Gyroscope.h"
#include "Fourbar/Txt_write.h"
#include "Fourbar/Grashof_condition.h"

int main() {
	double SIMULATION_TIME = 60;
	double SIM_TIME_STEP = 0.006;
	double GYROSCOPE_STDDEV = 0.01;
	double UPDATING_STDDEV_MOD = 10;
	double MOTION_STDDEV = 0.02;
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
	const bool TXT_WRITING_IS_ENABLED = false;
	const bool OUTPUT_IS_ENABLED = true;

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
		
		// Initialize the system and reference state.
		system.realizeTopology();
		SimTK::State RefState;

		// Create and add assembler.
		SimTK::Assembler assembler(system);
		assembler.setSystemConstraintsWeight(1);
		assembler.lockMobilizer(Bar1.getMobilizedBodyIndex());

		// We will random assign the reference state and particles states
		if (OUTPUT_IS_ENABLED) std::cout << FOURBAR_CONFIGURATION.get_description() 
			<< "\n\nAssigning states and assembling fourbar..." << std::endl;
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

		Stopwatch Sw_total_time(StopwatchMode::CPU_Time);
		Stopwatch Sw_ref_advance(StopwatchMode::CPU_Time);
		Stopwatch Sw_part_advance(StopwatchMode::CPU_Time);
		Stopwatch Sw_resample(StopwatchMode::CPU_Time);
		Sw_total_time.start();

		for (double time = 0; time <= SIMULATION_TIME; time += SIM_TIME_STEP) {	// Loop to slowly advance simulation
			
			if (OUTPUT_IS_ENABLED) {
				Sw_total_time.stop();
				std::cout << "\n NEXT ITERATION...\n\nCurrent real time: " << time << " s\nCurrent CPU time: "
					<< Sw_total_time.read()  << " s" << std::endl;
				Sw_total_time.start();
			}
			if (TXT_WRITING_IS_ENABLED) {
				Angle_write(RefState, FILTER.updParticleList());
				Omega_write(gyr, pargyr);
			}
			Sw_ref_advance.restart();
			Sw_ref_advance.start();
			advance(RefState, ts, SIM_TIME_STEP);					// Advance reference state
			Sw_ref_advance.stop();
			Sw_part_advance.restart();
			Sw_part_advance.start();
			FILTER.updateStates(ts, assembler);						// Advance particles state
			Sw_part_advance.stop();
			gyr.measure();											// Update reference gyroscope reading
			FILTER.updateWeights<Gyroscope>(gyr.read(), pargyr);	// Update particles gyroscope reading and weights
			FILTER.calculateESS();									// Calculate particles ESS

			if (OUTPUT_IS_ENABLED) {
				printf("\nPARTICLE SUMMARY:\n");
				for (std::size_t i = 0; i < PARTICLE_NUMBER; i++)
					printf("\nParticle %3.u \tOmega =%9.5f\tLog(w) = %10.5f\tAngle = %7.3f",
						static_cast<unsigned int>(i + 1),								static_cast<double>(pargyr[i].read()),
						static_cast<double>(FILTER.updParticleVec()[i].getWeight()),	static_cast<double>(to2Pi(FILTER.updParticleVec()[i].getState().getQ()[0]))
					);
				std::cout << "\n\nReference Gyroscope: " << gyr.read() << std::endl;
				std::cout << "Reference Angle: " << to2Pi(RefState.getQ()[0]) << std::endl;
				std::cout << "\nReference State advance time: " << Sw_ref_advance.read() <<  " s" << std::endl;
				std::cout << "Parrticles States advance time: " << Sw_part_advance.read() << " s" << std::endl;
				printf("\n ESS = %f %%\n", static_cast<double>(FILTER.getESS() * 100));
			}

			if (FILTER.getESS() < 0.5) {		// Resample when < 50 % of effective particles
				Sw_resample.start();
				FILTER.resample();
				Sw_resample.stop();
				for (std::size_t i = 0; i < PARTICLE_NUMBER; i++) {	// Noise added to angular velocity
					double Rate = Bar1.getRate(FILTER.updParticleVec()[i].updState());
					Bar1.setRate(FILTER.updParticleVec()[i].updState(), Rate + resample_noise.getValue());
				}
				if (OUTPUT_IS_ENABLED) {
					std::cout << "\nResample done! " << "Resample time: " << Sw_resample.read() << std::endl;
					Sw_resample.restart();
				}
			}

			if (OUTPUT_IS_ENABLED) {
				std::cout << "\nPress Enter to keep iterating" << std::endl;
				getchar();
			}
		}

		if (OUTPUT_IS_ENABLED) {
			std::cout << "\nSIMULATION ENDED. Press Enter to close this window." << std::endl;
			getchar();
		}
	}
	catch (const std::exception& e) {
		std::cout << "Error: " << e.what() << std::endl;
		getchar();
		return 1;
	}
}
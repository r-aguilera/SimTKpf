#include "Simbody.h"
#include "Fourbar/assemble_Fourbar.h"
#include "Fourbar/Grashof_condition.h"
#include "SimTKpf/Particle_Filter.h"
#include "SimTKpf/PF_utilities.h"

//TODO: assembly if input bar is not crank

void assemble_Fourbar(GrashofCondition& config, SimTK::MultibodySystem& system, SimTK::Assembler& assembler, SimTK::State& RefState, ParticleFilter& Filter) {
	double values[2];	// Assembly angle limit values 
	double limits[2];	// Limits for random angle function
	
	if (config.get_isCrank()) {	// If input bar is crank, assembly range is 0 to 2*Pi
		limits[0] = 0;
		limits[1] = 2 * SimTK::Pi;
	}
	else {
		bool thisAssemblySuccessed = false;		// True if fourbar can be assembled in the current iteration
		bool lastAssemblySuccessed = false;		// Stores previous thisAssemblySuccessed value
		bool is0rocker = false;					// True if fourbar can be assembled if angle is zero

		for (std::size_t n = 0, i = 0; n < 2; ++i) {
			// Try to assemble fourbar at 'i' degrees
			try {
				thisAssemblySuccessed = true;
				RefState = system.getDefaultState();
				RefState.updQ()[0] = i * SimTK::Pi / 180;
				assembler.assemble(RefState);
			}
			// Detect if assembly was not possible
			catch (const std::exception& e) {
				thisAssemblySuccessed = false;
			}

			if (i == 0) is0rocker = thisAssemblySuccessed;
			else 
				if (thisAssemblySuccessed != lastAssemblySuccessed) {	// Set values if there is a change in "assemblability"
					if (lastAssemblySuccessed)	values[n] = (i-1) * SimTK::Pi / 180;
					else						values[n] = i * SimTK::Pi / 180;
				++n;
				}
			lastAssemblySuccessed = thisAssemblySuccessed;
		}
		if (is0rocker) {	// Correct values before calling random function
			limits[0] = values[1] - 2 * SimTK::Pi;
			limits[1] = values[0];
		}
	}
	SimTK::Random::Uniform randomAngle(limits[0], limits[1]);
	randomAngle.setSeed(getSeed());

	// Assemble reference state
	RefState = system.getDefaultState();
	RefState.updQ()[0] = randomAngle.getValue();
	assembler.assemble(RefState);

	// Assemble particles states
	for (std::size_t i = 0; i < Filter.getParticleNumber(); ++i) {
		Filter.updParticleVec()[i].updState() = system.getDefaultState();
		Filter.updParticleVec()[i].updState().updQ()[0] = randomAngle.getValue();
		assembler.assemble(Filter.updParticleVec()[i].updState());
	}
}

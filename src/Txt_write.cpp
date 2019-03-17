#include <fstream>
#include "Simbody.h"
#include "PF_utilities.h"
#include "Txt_write.h"

void Angle_write(SimTK::State& ref_State, ParticleList& particles) {

	std::size_t particle_number = particles.getAllParticles().size();
	std::fstream File;
	SimTK::String filename = "Angles.txt";
	char AngleValue[9];
	
	File.open(filename, std::ios::app);
	sprintf_s(AngleValue, "%.4f", to2Pi(ref_State.getQ()[0]));
	File << AngleValue;
	
	for (std::size_t i = 0; i < particle_number; ++i) {
		sprintf_s(AngleValue, "\t%.4f", to2Pi(particles[i].getState().getQ()[0]));
		File << AngleValue;
	}

	File << std::endl;
	File.close();	
}

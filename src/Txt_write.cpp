#include <fstream>
#include "Simbody.h"
#include "PF_utilities.h"
#include "Txt_write.h"

void Angle_write(SimTK::State& ref_State, ParticleList& particles) {
	std::fstream File;
	SimTK::String filename = "Angles.txt";
	std::size_t particle_number = particles.getAllParticles().size();

	File.open(filename, std::ios::app);
	File << ref_State.getQ()[0];
	
	for (std::size_t i = 0; i < particle_number; ++i) {
		File << "\t" << particles[i].getState().getQ()[0];
	}
	File << std::endl;
	File.close();	
}

#include <fstream>
#include "Simbody.h"
#include "SimTKpf/PF_utilities.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKcommon/internal/Subsystem.h"
#include "Fourbar/Gyroscope.h"
#include "Fourbar/Txt_write.h"

void Angle_write(SimTK::State& ref_State, ParticleList& particles) {

	std::size_t particle_number = particles.getAllParticles().size();
	std::fstream File;
	SimTK::String filename = "Angles.txt";
	char AngleValue[9];
	
	File.open(filename, std::ios::app);
	sprintf(AngleValue, "%.4f", to2Pi(ref_State.getQ()[0]));
	File << AngleValue;
	
	for (std::size_t i = 0; i < particle_number; ++i) {
		sprintf(AngleValue, "\t%.4f", to2Pi(particles[i].getState().getQ()[0]));
		File << AngleValue;
	}

	File << std::endl;
	File.close();	
}

void Omega_write(Gyroscope& RefGyr, std::vector<Gyroscope>& PtGyr) {

	std::size_t particle_number = PtGyr.size();
	std::fstream File;
	SimTK::String filename = "Omegas.txt";
	char OmegaValue[8];

	File.open(filename, std::ios::app);
	RefGyr.measure();
	sprintf(OmegaValue, "%.3f", RefGyr.read());
	File << OmegaValue;

	for (std::size_t i = 0; i < particle_number; ++i) {
		PtGyr[i].measure();
		sprintf(OmegaValue, "\t%.3f", PtGyr[i].read());
		File << OmegaValue;
	}

	File << std::endl;
	File.close();
}

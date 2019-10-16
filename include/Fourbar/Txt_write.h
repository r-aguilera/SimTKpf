#ifndef _TXT_WRITE_
#define _TXT_WRITE_

#include "Simbody.h"
#include "SimTKpf/PF_utilities.h"
#include "SimTKpf/Particle_Classes.h"
#include "Fourbar/Gyroscope.h"

// Functions to write data to txt files

void Angle_write(SimTK::State&, ParticleList&);
void Weight_write(ParticleList&);
void Omega_write(Gyroscope&, std::vector<Gyroscope>&);

#endif
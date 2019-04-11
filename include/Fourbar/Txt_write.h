#ifndef _TXT_WRITE_
#define _TXT_WRITE_

#include "Simbody.h"
#include "SimTKpf/PF_utilities.h"
#include "Fourbar/Gyroscope.h"

// This will write reference and particles both angle into an txt file:

void Angle_write(SimTK::State&, ParticleList&);
void Omega_write(Gyroscope&, std::vector<Gyroscope>&);

#endif
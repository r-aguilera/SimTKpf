#ifndef _ASSEMBLE_FOURBAR_
#define _ASSEMBLE_FOURBAR_

#include "Simbody.h"
#include "Fourbar/Grashof_condition.h"
#include "SimTKpf/Particle_Filter.h"
#include "SimTKpf/PF_utilities.h"

// Assemble fourbar at a random yet attainable angle
void assemble_Fourbar(GrashofCondition&, SimTK::MultibodySystem&, SimTK::Assembler&, SimTK::State&, ParticleFilter&);

#endif

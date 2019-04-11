#include "Simbody.h"
#include "SimTKpf/Simbody_Instrument.h"
#include "Fourbar/Gyroscope.h"

Gyroscope::Gyroscope(const SimTK::MultibodySystem& system, const SimTK::SimbodyMatterSubsystem& matter, SimTK::State& RefState,
	double GroundBarLenght, double StdDev = 0) : Simbody_Instrument(system, matter, RefState, StdDev),
	m_GroundBarLenght(GroundBarLenght) {}

void Gyroscope::measure(){
	using namespace SimTK;
	m_system.realize(rf_state, Stage::Velocity);	// We can get now cartesian coordinates and velocities

	// Get first bar mobilized body
	MobilizedBodyIndex index = MobilizedBodyIndex(1);
	MobilizedBody body = m_matter.getMobilizedBody(index);

	// Get positions
	Vec3 Pos2 = body.getBodyOriginLocation(rf_state);
	Vec3 Pos1 = Vec3(-m_GroundBarLenght, 0, 0);

	// Get velocities
	Vec3 Vel2 = body.getBodyOriginVelocity(rf_state);
	Vec3 Vel1 = Vec3(0, 0, 0);

	Vec3 r = Pos2 - Pos1;
	Real distance = r.norm();
	Vec3 w = -((Vel2 - Vel1) % r) / (distance * distance);
	Vec3 u = Vec3(0, 0, 1);

	reading = ~w * u;	// Dot product
}


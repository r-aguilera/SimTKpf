#include "Simbody.h"
#include "SimTKpf/Particle_Classes.h"
#include "SimTKpf/PF_utilities.h"

void advance(SimTK::State& state, SimTK::TimeStepper& ts, const double dt) {
	ts.initialize(state);
	ts.stepTo(ts.getTime() + dt);
	state = ts.getState();

}

double NormalProb(const double x, const double mean, const double s) {
	double y = ((mean - x) / s);
	return -0.5*y*y;
}
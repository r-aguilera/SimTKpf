#include "Simbody.h"
#include "SimTKpf/Measuring_Instrument.h"
#include "SimTKpf/Stopwatch.h"

// Stopwatch Implementation

Stopwatch::Stopwatch(StopwatchMode SWmode) : mode(SWmode) { reading = 0; }

void Stopwatch::start() {
	if (mode == StopwatchMode::Real_Time)		ref_time = SimTK::realTime();
	else if (mode == StopwatchMode::CPU_Time)	ref_time = SimTK::cpuTime();
}
void Stopwatch::restart() { reading = 0; }

void Stopwatch::stop() {
	measure();
}

void Stopwatch::measure() {
	if (mode == StopwatchMode::Real_Time)		reading += SimTK::realTime() - ref_time;
	else if (mode == StopwatchMode::CPU_Time)	reading += SimTK::cpuTime() - ref_time;
}


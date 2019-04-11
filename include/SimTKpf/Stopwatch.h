#ifndef _STOPWATCH_
#define _STOPWATCH_

#include "Simbody.h"
#include "SimTKpf/Measuring_Instrument.h"

// Mode variable to decide if measuring real time or CPU time:

enum class StopwatchMode : bool {Real_Time, CPU_Time};

// Class that actually measure time. Member reading is the time elapsed between start() and stop() functions

class Stopwatch : public Measuring_Instrument {
public:
	Stopwatch(StopwatchMode);	// Constructor given the mode
	void start();		// Initiate or reanude the count (take a time reference)
	void restart();		// Restart the count (reading set to zero)
	void stop();		// Stop the count (update the reading)
	void measure() override;	// Just stop() implementation

private:
	double ref_time;		// Zero time reference from a start() call
	StopwatchMode mode;
};

#endif

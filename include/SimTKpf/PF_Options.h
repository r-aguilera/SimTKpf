#ifndef _PF_OPTIONS_
#define _PF_OPTIONS_

#include <exception>

// Auxiliary enumeration class for accesing a single particle filter option:
enum class PF_Options_index {
	SIMULATION_TIME_STEP,
	SENSOR_STDDEV,
	SENSOR_STDDEV_MOD,
	MOTION_STDDEV,
	RESAMPLE_STDDEV
};

// Exception in case of unexpected behaviour using PF_Option_index class:
class PF_Options_exception : public std::exception {
	virtual const char* what() const throw(){
		return "PF_Option_index used does not match existing ones. \nDefault statement reached in switch statements of ParticleFilter.cpp methods:\nsetOption, getOption or updOption";
	}
};

// Options class for particle filter algorithm:
class PF_Options {
public:
	PF_Options();
	// 1 - Time step between two filter iterations
	// 2 - Standard deviation of Ground Truth sensor's noise 
	// 3 - Decimal who modifies the sensor standard deviation during updating stage
	// 4 - Standard deviation of noise added in prediction stage
	// 5 - Standard deviation of noise added after the resampling stage
	PF_Options(double, double, double, double, double);

	void setOptions(double, double, double, double, double);
	void operator =(PF_Options);

	void setOption(PF_Options_index, double);		// Write single option value
	const double getOption(PF_Options_index) const;	// Return single option as read-only double value
	double& updOption(PF_Options_index);			// Return single option as writable double value

private:
	double SIMULATION_TIME_STEP;
	double SENSOR_STDDEV;
	double SENSOR_STDDEV_MOD;
	double MOTION_STDDEV;
	double RESAMPLE_STDDEV;	
};

#endif

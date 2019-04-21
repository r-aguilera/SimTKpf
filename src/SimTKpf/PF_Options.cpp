#include <exception>
#include "SimTKpf/PF_Options.h"

// PF_Options Implementation

PF_Options::PF_Options(	
	double PF_SIMULATION_TIME_STEP,		// Time step between two filter iterations
	double PF_SENSOR_STDDEV,			// Standard deviation of Ground Truth sensor's noise 
	double PF_SENSOR_STDDEV_MOD,		// Decimal who modifies the sensor standard deviation during updating stage
	double PF_MOTION_STDDEV,			// Standard deviation of noise added in prediction stage
	double PF_RESAMPLE_STDDEV			// Standard deviation of noise added after the resampling stage
	) :	SIMULATION_TIME_STEP(PF_SIMULATION_TIME_STEP),
		SENSOR_STDDEV(PF_SENSOR_STDDEV),
		SENSOR_STDDEV_MOD(PF_SENSOR_STDDEV_MOD),
		MOTION_STDDEV(PF_MOTION_STDDEV),
		RESAMPLE_STDDEV(PF_RESAMPLE_STDDEV) {}

void PF_Options::setOptions(double PF_SIMULATION_TIME_STEP,double PF_SENSOR_STDDEV, double PF_SENSOR_STDDEV_MOD,
	double PF_MOTION_STDDEV, double PF_RESAMPLE_STDDEV){
		SIMULATION_TIME_STEP = PF_SIMULATION_TIME_STEP;
		SENSOR_STDDEV = PF_SENSOR_STDDEV;
		SENSOR_STDDEV_MOD = PF_SENSOR_STDDEV_MOD;
		MOTION_STDDEV = PF_MOTION_STDDEV;
		RESAMPLE_STDDEV = PF_RESAMPLE_STDDEV;
}

void PF_Options::operator =(PF_Options PF_Options2) {
	SIMULATION_TIME_STEP = PF_Options2.SIMULATION_TIME_STEP;
	SENSOR_STDDEV = PF_Options2.SENSOR_STDDEV;
	SENSOR_STDDEV_MOD = PF_Options2.SENSOR_STDDEV_MOD;
	MOTION_STDDEV = PF_Options2.MOTION_STDDEV;
	RESAMPLE_STDDEV = PF_Options2.RESAMPLE_STDDEV;
}

void PF_Options::setOption(PF_Options_index index, double newValue) {
	switch (index) {
		case PF_Options_index::SIMULATION_TIME_STEP:	SIMULATION_TIME_STEP	= newValue;		break;
		case PF_Options_index::SENSOR_STDDEV:			SENSOR_STDDEV			= newValue;		break;
		case PF_Options_index::SENSOR_STDDEV_MOD:		SENSOR_STDDEV_MOD		= newValue;		break;
		case PF_Options_index::MOTION_STDDEV:			MOTION_STDDEV			= newValue;		break;
		case PF_Options_index::RESAMPLE_STDDEV:			RESAMPLE_STDDEV			= newValue;		break;
		default: throw PF_Options_exception();
	}	
}

const double PF_Options::getOption(PF_Options_index index) const {
	switch (index) {
		case PF_Options_index::SIMULATION_TIME_STEP:	return SIMULATION_TIME_STEP;	break;
		case PF_Options_index::SENSOR_STDDEV:			return SENSOR_STDDEV;			break;
		case PF_Options_index::SENSOR_STDDEV_MOD:		return SENSOR_STDDEV_MOD;		break;
		case PF_Options_index::MOTION_STDDEV:			return MOTION_STDDEV;			break;
		case PF_Options_index::RESAMPLE_STDDEV:			return RESAMPLE_STDDEV;			break;
		default: throw PF_Options_exception();
	}
}

double& PF_Options::updOption(PF_Options_index index) {
	switch (index) {
		case PF_Options_index::SIMULATION_TIME_STEP:	return SIMULATION_TIME_STEP;	break;
		case PF_Options_index::SENSOR_STDDEV:			return SENSOR_STDDEV;			break;
		case PF_Options_index::SENSOR_STDDEV_MOD:		return SENSOR_STDDEV_MOD;		break;
		case PF_Options_index::MOTION_STDDEV:			return MOTION_STDDEV;			break;
		case PF_Options_index::RESAMPLE_STDDEV:			return RESAMPLE_STDDEV;			break;
		default: throw PF_Options_exception();
	}
}

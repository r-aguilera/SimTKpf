#ifndef _MEASURING_INSTRUMENT_
#define _MEASURING_INSTRUMENT_

// General class for virtual ideal measuring instruments:

class Measuring_Instrument {
public:
	virtual void measure()=0;	// Update instrument reading. Pure virtual, must be overriden
	virtual const double read();	// Return instrument reading

protected:
	double reading;
};

#endif
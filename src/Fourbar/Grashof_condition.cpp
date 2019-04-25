#include "Simbody.h"
#include "Fourbar/Grashof_condition.h"

bool evaluateGrashof(std::vector<double>& L) {
	bool isInputBarACrank;
	double T1, T2, T3;

	T1 = L[1] + L[3] - L[4] - L[2];
	T2 = L[1] + L[4] - L[3] - L[2];
	T3 = L[4] + L[3] - L[1] - L[2];

	// TODO: Grashof condition evaluation according to terms T1, T2 & T3.
}

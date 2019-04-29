#include "Simbody.h"
#include "Fourbar/Grashof_condition.h"

bool evaluateGrashof(std::vector<double>& L) {
	bool isInputBarACrank;
	double T1, T2, T3;

	T1 = L[1] + L[3] - L[4] - L[2];
	T2 = L[1] + L[4] - L[3] - L[2];
	T3 = L[4] + L[3] - L[1] - L[2];

	// Grashof condition evaluation according to terms T1, T2 & T3.
	if (T1*T2*T3 == 0) {
		std::cout << "Fourbar is a Crank-Crank and it folds." << std::endl;
		isInputBarACrank = true;
	} 
	else {
		if (T1 < 0) {
			if (T2 < 0) {
				if (T3 < 0) {
					std::cout << "Fourbar is a 0-Rocker-0-Rocker." << std::endl;
					isInputBarACrank = false;
				}
				else {
					std::cout << "Fourbar is a Crank-Crank." << std::endl;
					isInputBarACrank = true;
				}
			}
			else {
				if (T3 < 0) {
					std::cout << "Fourbar is a Rocker-Rocker." << std::endl;
					isInputBarACrank = false;
				}
				else {
					std::cout << "Fourbar is a Pi-Rocker-Pi-Rocker." << std::endl;
					isInputBarACrank = false;
				}
			}
		}
		else {
			if (T2 < 0) {
				if (T3 < 0) {
					std::cout << "Fourbar is a Rocker-Crank." << std::endl;
					isInputBarACrank = false;
				}
				else {
					std::cout << "Fourbar is a Pi-Rocker-0-Rocker." << std::endl;
					isInputBarACrank = false;
				}
			}
			else {
				if (T3 < 0) {
					std::cout << "Fourbar is a 0-Rocker-Pi-Rocker." << std::endl;
					isInputBarACrank = false;
				}
				else {
					std::cout << "Fourbar is a Crank-Rocker." << std::endl;
					isInputBarACrank = true;
				}
			}
		}
	}

	return isInputBarACrank;
}

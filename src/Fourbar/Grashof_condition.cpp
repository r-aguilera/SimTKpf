#include <string>
#include "Simbody.h"
#include "Fourbar/Grashof_condition.h"

// GrashofCondition Implementation
GrashofCondition::GrashofCondition(std::string description, bool isCrank) : description(description), isCrank(isCrank) {}
std::string GrashofCondition::get_description() { return description;	}
bool GrashofCondition::get_isCrank()			{ return isCrank;		}

// evaluateGrashof function Implementation
GrashofCondition evaluateGrashof(std::vector<double>& L) {
	const double T1 = L[0] + L[2] - L[3] - L[1];
	const double T2 = L[0] + L[3] - L[2] - L[1];
	const double T3 = L[3] + L[2] - L[0] - L[1];

	const bool T1pos = (T1 > 0 ? 1 : 0);
	const bool T2pos = (T2 > 0 ? 1 : 0);
	const bool T3pos = (T3 > 0 ? 1 : 0);

	GrashofCondition AllConditions[9]{
		{ "Fourbar is a 0-Rocker-0-Rocker.",		false	},
		{ "Fourbar is a Crank-Crank.",				true	},
		{ "Fourbar is a Rocker-Rocker.",			false	},
		{ "Fourbar is a Pi-Rocker-Pi-Rocker.",		false	},
		{ "Fourbar is a Rocker-Crank.",				false	},
		{ "Fourbar is a Pi-Rocker-0-Rocker.",		false	},
		{ "Fourbar is a 0-Rocker-Pi-Rocker.",		false	},
		{ "Fourbar is a Crank-Rocker.",				true	},
		{ "Fourbar is a Crank-Crank and it folds.",	true	}
	};

	std::size_t chosenCondition;

	// Grashof condition evaluation according to terms T1, T2 & T3.
	if (T1*T2*T3 != 0) chosenCondition = T1pos * 4 + T2pos * 2 + T3pos;
	else chosenCondition = 8;
	
	return AllConditions[chosenCondition];
}

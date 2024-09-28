#pragma once

#include "types.hpp"
#include "Params/params.h"

namespace mpcc {

class Models {
public:
	Models(const Car& car, const Tire& tire);

	State calculateCombinedSlipDynamicModelDerivatives(const State& state, const Input& input) const;
	State calculateSimpleCombinedModelDerivatives(const State& state, const Input& input) const;
	State calculateSimpleDynamicModelDrivatives(const State& state, const Input& input) const;
	State calculateKinematicModelDerivatives(const State& state, const Input& input) const;

	void unwrapState(State& state) const;
	
private:
	std::pair <double, double> calculateTireForces(double sa, double kappa, double Fz, double Dfz, double fzNominal) const;
	State ode4(const State& state, const Input& input, double ts, State calculateDerivatives) const;

private:
	Car d_car;
	Tire d_tire;
};

} // namespace mpcc
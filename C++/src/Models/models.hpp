#pragma once

#include "types.hpp"
#include "Params/params.hpp"

namespace mpcc
{

	class Models
	{
	public:
		Models(const PathToJson &jsonPath);

		State13 calculateCombinedSlipDynamicModelDerivatives(const State13 &state, const Input &input) const;
		State calculateSimpleCombinedModelDerivatives(const State &state, const Input &input) const;
		State calculateSimpleDynamicModelDerivatives(const State &state, const Input &input) const;
		State calculateKinematicModelDerivatives(const State &state, const Input &input) const;

		template <typename S, typename F>
		S ode4(const S &state, const Input &input, double ts, F calculateDerivatives) const
		{
			// 4th order Runge Kutta (RK4) implementation
			// 4 evaluation points of continuous dynamics
			// evaluating the 4 points
			S k1 = calculateDerivatives(state, input);
			S k2 = calculateDerivatives(state + ts / 2. * k1, input);
			S k3 = calculateDerivatives(state + ts / 2. * k2, input);
			S k4 = calculateDerivatives(state + ts * k3, input);
			// combining to give output
			return state + ts * (k1 / 6. + k2 / 3. + k3 / 3. + k4 / 6.);
		}

	private:
		std::pair<double, double> calculateTireForces(double alpha, double kappa, double Fz, double Dfz, double fzNominal) const;
		double sign(double value) const;

	private:
		Car d_car;
		Tire d_tire;
	};

} // namespace mpcc
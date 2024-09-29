#include "models.hpp"

namespace mpcc
{

	Models::Models(const PathToJson &jsonPath) : d_car(jsonPath.carPath), d_tire(jsonPath.tirePath)
	{
	}

	State13 Models::calculateCombinedSlipDynamicModelDerivatives(const State13 &state, const Input &input) const
	{
		// Dynamic forces
		double rDyn = d_tire.R;

		double gearRatio = d_car.gearRatio;
		double brakesRatio = d_car.brakesRatio;

		double m = d_car.m;
		double iz = d_car.iz;
		double lf = d_car.lf;
		double lr = d_car.lr;
		double iw = d_tire.I;
		double gAcc = d_car.g;
		double fzNominal = d_tire.fzNominal;

		// normal load on the one front wheel
		double Ffz = lr * m * gAcc / (2. * (lf + lr));
		double Dffz = (Ffz - fzNominal) / fzNominal;

		// normal load on the one rear wheel
		double Frz = lf * m * gAcc / (2. * (lf + lr));
		double Drfz = (Frz - fzNominal) / fzNominal;

		// rolling resistance of the two front wheels
		double Ffrr = 2. * d_tire.QSY1 * Ffz * std::tanh(state(vx));
		// Ffrr = 200*tanh(vx);

		// rolling resistance of the two rear wheels
		double Frrr = 2. * d_tire.QSY1 * Frz * std::tanh(state(vx));
		// Frrr = 200*tanh(vx);

		// brakes front force
		double Fbf = (-state(brakes) * brakesRatio / (1. + brakesRatio)) / rDyn * std::tanh(state(vx));

		// brakes rear force
		double Fbr = (-state(brakes) * 1. / (1. + brakesRatio)) / rDyn * std::tanh(state(vx));

		// drivetrain force
		double Fdrv = (gearRatio * state(throttle)) / rDyn;

		// slip angle of the front wheel
		double saf0 = std::atan2((state(vy) + state(r) * lf), vx) - state(steeringAngle);

		// slip angle of the rear wheel
		double sar0 = std::atan2((state(vy) - state(r) * lr), state(vx));

		// slip ratio of the front wheel
		double vlf = (state(vy) + state(r) * lf) * std::sin(state(steeringAngle)) + state(vx) * std::cos(state(steeringAngle));
		double kappaf0 = (state(omegaf) * rDyn - vlf) / (std::max(1., vlf));

		// slip ratio of the rear wheel
		double kappar0 = (state(omegar) * rDyn - state(vx)) / (std::max(1., state(vx)));

		auto [Ffy0, Ffx0] = calculateTireForces(saf0, kappaf0, Ffz, Dffz, fzNominal);
		auto [Fry0, Frx0] = calculateTireForces(sar0, kappar0, Frz, Drfz, fzNominal);
		double Ffy = 2. * Ffy0;
		double Ffx = 2. * Ffx0;
		double Fry = 2. * Fry0;
		double Frx = 2. * Frx0;
		// drag force
		double Fdrag = d_car.cd * std::pow(state(vx), 2.);

		return {state(vx) * cos(state(yaw)) - state(vy) * sin(state(yaw)),
						vx * sin(state(yaw)) + state(vy) * cos(state(yaw)),
						r,
						1 / m *
								(Frx + cos(state(steeringAngle)) * Ffx + Fdrag - sin(state(steeringAngle)) * Ffy +
								 m * state(vy) * state(r)),
						1 / m *
								(Fry + cos(state(steeringAngle)) * Ffy + sin(state(steeringAngle)) * Ffx - m * state(vx) * state(r)),
						1 / iz *
								(-Fry * lr + (cos(state(steeringAngle)) * Ffy + sin(state(steeringAngle)) * Ffx) * lf),
						state(vs),
						input(dThrottle),
						input(dSteeringAngle),
						input(dBrakes),
						input(dVs),
						-(Ffx - Fbf - Ffrr) / 2 * rDyn / iw,
						(Fdrv + Fbr - Frx + Frrr) / 2 * rDyn / iw};
	}

	State Models::calculateSimpleCombinedModelDerivatives(const State &state, const Input &input) const
	{
		std::cout << "In model" << std::endl;
		
		State dynamicDerivs = calculateSimpleDynamicModelDerivatives(state, input);
		
		State kinematicDerivs = calculateKinematicModelDerivatives(state, input);

		

		double lambda = std::min(std::max((state(vx) - 3.) / 2., 0.), 1.);

		return {state(vx) * cos(state(yaw)) - state(vy) * sin(state(yaw)),
						state(vx) * sin(state(yaw)) + state(vy) * cos(state(yaw)),
						state(r),
						lambda * dynamicDerivs(vx) + (1. - lambda) * kinematicDerivs(vx),
						lambda * dynamicDerivs(vy) + (1. - lambda) * kinematicDerivs(vy),
						lambda * dynamicDerivs(r) + (1. - lambda) * kinematicDerivs(r),
						state(vs),
						input(dThrottle),
						input(dSteeringAngle),
						input(dBrakes),
						input(dVs)};
	}

	State Models::calculateSimpleDynamicModelDerivatives(const State &state, const Input &input) const
	{
		double m = d_car.m;
		double iz = d_car.iz;
		double lf = d_car.lf;
		double lr = d_car.lr;
		double gAcc = d_car.g;
		double gearRatio = d_car.gearRatio;
		double brakesRatio = d_car.brakesRatio;

		double rDyn = d_tire.R;

		// normal load on the one front wheel
		double Ffz = lr * m * gAcc / (2.0 * (lf + lr));

		// normal load on the one rear wheel
		double Frz = lf * m * gAcc / (2.0 * (lf + lr));

		// rolling resistance of the two front wheels
		double Ffrr = 2 * d_tire.QSY1 * Ffz * tanh(state(vx));

		// rolling resistance of the two rear wheels
		double Frrr = 2 * d_tire.QSY1 * Frz * tanh(state(vx));

		// brakes front force
		double Fbf = (-state(brakes) * brakesRatio / (1 + brakesRatio)) / rDyn * tanh(state(vx));

		// brakes rear force
		double Fbr = (-state(brakes) * 1 / (1 + brakesRatio)) / rDyn * tanh(state(vx));

		// drivetrain force
		double Fdrv = (gearRatio * state(throttle)) / rDyn;

		// longitudinal front force
		double Ffx = Fbf + Ffrr;

		// longitudinal rear force
		double Frx = Fbr + Fdrv + Frrr;

		// slip angle of the front wheel
		double saf = atan2((state(vy) + state(r) * lf), state(vx)) - state(steeringAngle);

		// slip angle of the rear wheel
		double sar = atan2((state(vy) - state(r) * lr), state(vx));

		// latteral front force
		double Ffy = -saf * d_tire.Cy;

		// latteral rear force
		double Fry = -sar * d_tire.Cy;

		// drag force
		double Fdrag = d_car.cd * std::pow(state(vx), 2.0);

		return {state(vx) * cos(state(yaw)) - state(vy) * sin(state(yaw)),
						state(vx) * sin(state(yaw)) + state(vy) * cos(state(yaw)),
						state(r),
						1. / m * (Frx + cos(state(steeringAngle)) * Ffx + Fdrag - sin(state(steeringAngle)) * Ffy + m * state(vy) * state(r)),
						1. / m * (Fry + cos(state(steeringAngle)) * Ffy + sin(state(steeringAngle)) * Ffx - m * state(vx) * state(r)),
						1. / iz * (-Fry * lr + (cos(state(steeringAngle)) * Ffy + sin(state(steeringAngle)) * Ffx) * lf),
						state(vs),
						input(dThrottle),
						input(dSteeringAngle),
						input(dBrakes),
						input(dVs)};
	}

	State Models::calculateKinematicModelDerivatives(const State &state, const Input &input) const
	{
		// d_car parameters
		double rDyn = d_tire.R;

		double gearRatio = d_car.gearRatio;
		double brakesRatio = d_car.brakesRatio;

		double m = d_car.m;
		double lf = d_car.lf;
		double lr = d_car.lr;
		double gAcc = d_car.g;

		// normal load on the one front wheel
		double Ffz = lr * m * gAcc / (2.0 * (lf + lr));

		// normal load on the one rear wheel
		double Frz = lf * m * gAcc / (2.0 * (lf + lr));

		// rolling resistance of the two front wheels
		double Ffrr = 2 * d_tire.QSY1 * Ffz * tanh(state(vx));

		// rolling resistance of the two rear wheels
		double Frrr = 2 * d_tire.QSY1 * Frz * tanh(state(vx));

		// brakes front force
		double Fbf = (-state(brakes) * brakesRatio / (1 + brakesRatio)) / rDyn * tanh(state(vx));

		// brakes rear force
		double Fbr = (-state(brakes) * 1 / (1 + brakesRatio)) / rDyn * tanh(state(vx));

		// drivetrain force
		double Fdrv = (gearRatio * state(throttle)) / rDyn;

		// longitudinal front force
		double Ffx = Fbf + Ffrr;

		// longitudinal rear force
		double Frx = Fbr + Fdrv + Frrr;

		// drag force
		double Fdrag = d_car.cd * std::pow(state(vx), 2.);

		// dot vx
		double vxDot = 1 / m * (Frx + cos(state(steeringAngle)) * Ffx + Fdrag);

		return {state(vx) * cos(state(yaw)) - state(vy) * sin(state(yaw)),
						state(vx) * sin(state(yaw)) + state(vy) * cos(state(yaw)),
						state(r),
						vxDot,
						lr / (lr + lf) * (input(dSteeringAngle) * state(vx) + state(steeringAngle) * vxDot),
						1. / (lr + lf) * (input(dSteeringAngle) * state(vx) + state(steeringAngle) * vxDot),
						state(vs),
						input(dThrottle),
						input(dSteeringAngle),
						input(dBrakes),
						input(dVs)};
	}

	std::pair<double, double> Models::calculateTireForces(double alpha, double kappa, double Fz, double Dfz, double fzNominal) const
	{
		// latteral tire force Pacejka(Combined slip)
		double SHy = (d_tire.PHY1 + d_tire.PHY2 * Dfz) * d_tire.LHY;
		double SVy = Fz * ((d_tire.PVY1 + d_tire.PVY2 * Dfz) * d_tire.LVY) * d_tire.LMUY;

		double Ky = d_tire.PKY1 * fzNominal * std::sin(2.0 * std::atan2(Fz, (d_tire.PKY2 * fzNominal * d_tire.LFZO))) * d_tire.LFZO * d_tire.LKY;

		double muy = (d_tire.PDY1 + d_tire.PDY2 * Dfz) * d_tire.LMUY;
		double Dy = muy * Fz;

		double Cy = d_tire.PCY1 * d_tire.LCY;

		double By = Ky / (Cy * Dy);

		double Ey = (d_tire.PEY1 + d_tire.PEY2 * Dfz) * d_tire.LEY;

		// combined latteral force
		double sa = alpha + SHy;
		double Fy0 = Dy * std::sin(Cy * std::atan(By * sa - Ey * (By * sa - std::atan(By * sa)))) + SVy;
		double Byk = d_tire.RBY1 * std::cos(std::atan(d_tire.RBY2 * (alpha - d_tire.RBY3))) * d_tire.LKY;
		double Cyk = d_tire.RCY1;
		double Eyk = d_tire.REY1 + d_tire.REY2 * Dfz;
		double SHyk = d_tire.RHY1 + d_tire.RHY2 * Dfz;
		double Dyk = Fy0 / (cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk)))));
		double DVyk = muy * Fz * (d_tire.RVY1 + d_tire.RVY2 * Dfz) * cos(atan(Byk * SHyk));
		double kappa_s = kappa + SHyk;
		double SVyk = DVyk * sin(d_tire.RVY5 * atan(d_tire.RVY6 * kappa)) * d_tire.LVYKA;

		double Fy = Dyk * cos(Cyk * atan(Byk * kappa_s - Eyk * (Byk * kappa_s - atan(Byk * kappa_s)))) + SVyk;

		// longitudinal tire force Pacejka(Combined slip)
		double SHx = (d_tire.PHX1 + d_tire.PHX2 * Dfz) * d_tire.LHX;
		double SVx = Fz * (d_tire.PVX1 + d_tire.PVX2 * Dfz) * d_tire.LVX * d_tire.LMUX;
		double Kx = Fz * (d_tire.PKX1 + d_tire.PKX2 * Dfz) * exp(d_tire.PKX3 * Dfz) * d_tire.LKX;
		double mux = (d_tire.PDX1 + d_tire.PDX2 * Dfz);
		double Cx = d_tire.PCX1 * d_tire.LCX;
		double Dx = mux * Fz;
		double Bx = Kx / (Cx * Dx);
		// combined longitudinal force
		double kappax = kappa + SHx;
		double Ex = (d_tire.PEX1 + d_tire.PEX2 * Dfz + d_tire.PEX3 * Dfz * Dfz) * (1 - d_tire.PEX4 * sign(kappax)) * d_tire.LEX;
		double Fx0 = Dx * sin(Cx * atan(Bx * kappax - Ex * (Bx * kappax - atan(Bx * kappax)))) + SVx;

		double Bxa = d_tire.RBX1 * cos(atan(d_tire.RBX2 * kappa)) * d_tire.LXAL;
		double Cxa = d_tire.RCX1;
		double Exa = d_tire.REX1 + d_tire.REX2 * Dfz;
		double SHxa = d_tire.RHX1;
		double Dxa = Fx0 / (cos(Cxa * atan(Bxa * SHxa - Exa * (Bxa * SHxa - atan(Bxa * SHxa)))));
		double alpha_s = alpha + SHxa;
		double Fx = Dxa * cos(Cxa * atan(Bxa * alpha_s - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s))));

		return {Fy, Fx};
	}

	double Models::sign(double value) const
	{
		return (value >= 0.) ? 1 : -1;
	}
} // namespace mpcc
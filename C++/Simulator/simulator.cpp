#include "simulator.hpp"

namespace mpcc
{
State Simulator::calculateDerivatives(const State& state, const Input& input) const{
  // Dynamic forces
  double rDyn = tire.R;
  
  double gearRatio = car.gearRatio;
  double brakesRatio = car.brakesRatio;

  double m = car.m;
  double iz = car.iz;
  double lf = car.lf;
  double lr = car.lr;
  double iw = tire.I;
  double gAcc = car.g;
  double fzNominal = tire.fzNominal;

  // normal load on the one front wheel
  double Ffz = lr*m*gAcc/(2.*(lf+lr));
  double Dffz = (Ffz-fzNominal)/fzNominal;

  // normal load on the one rear wheel
  double Frz = lf*m*gAcc/(2.*(lf+lr));
  double Drfz = (Frz-fzNominal)/fzNominal;
  
  // rolling resistance of the two front wheels
  double Ffrr = 2.*tire.QSY1*Ffz*std::tanh(state(vx));
  // Ffrr = 200*tanh(vx);

  // rolling resistance of the two rear wheels
  double Frrr = 2.*tire.QSY1*Frz*std::tanh(state(vx));
  // Frrr = 200*tanh(vx);
  
  // brakes front force
  double Fbf = (-state(brakes)*brakesRatio/(1. + brakesRatio))/rDyn*std::tanh(state(vx));

  // brakes rear force
  double Fbr = (-state(brakes)*1./(1. + brakesRatio))/rDyn*std::tanh(state(vx));

  // drivetrain force
  double Fdrv = (gearRatio*state(throttle))/rDyn;

  // slip angle of the front wheel
  double saf0 = std::atan2((state(vy)+state(r)*lf),vx)-state(steeringAngle);

  // slip angle of the rear wheel
  double sar0 = std::atan2((state(vy)-state(r)*lr),state(vx));

  // slip ratio of the front wheel
  double vlf = (state(vy)+state(r)*lf)*std::sin(state(steeringAngle))+state(vx)*std::cos(state(steeringAngle));
  double kappaf0 = (state(omegaf) * rDyn - vlf) / (std::max(1., vlf));

  // slip ratio of the rear wheel
  double kappar0 = (state(omegar) * rDyn - state(vx)) / (std::max(1., state(vx)));

  auto [Ffy0,Ffx0] = combinedSlipTireModel(saf0,kappaf0,Ffz,Dffz,fzNominal);
  auto [Fry0,Frx0] = combinedSlipTireModel(sar0,kappar0,Frz,Drfz,fzNominal);
  double Ffy = 2. * Ffy0;
  double Ffx = 2. * Ffx0;
  double Fry = 2. * Fry0;
  double Frx = 2. * Frx0;
  // drag force
  double Fdrag = car.cd*std::pow(state(vx),2.);

  return  {state(vx)*cos(state(yaw))-state(vy)*sin(state(yaw)),
          vx*sin(state(yaw))+state(vy)*cos(state(yaw)),
          r,
          1/m*
          (Frx+cos(state(steeringAngle))*Ffx+Fdrag-sin(state(steeringAngle))*Ffy +
          m*state(vy)*state(r)),
          1/m*
          (Fry+cos(state(steeringAngle))*Ffy+sin(state(steeringAngle))*Ffx-m*state(vx)*state(r)),
          1/iz *
          (-Fry*lr+(cos(state(steeringAngle))*Ffy+sin(state(steeringAngle))*Ffx)*lf),
          state(vs),
          input(dThrottle),
          input(dSteeringAngle),
          input(dBrakes),
          input(dVs),
          -(Ffx-Fbf-Ffrr)/ 2 * rDyn / iw,
          (Fdrv + Fbr - Frx + Frrr) / 2 * rDyn / iw};
}

} // namespace mpcc
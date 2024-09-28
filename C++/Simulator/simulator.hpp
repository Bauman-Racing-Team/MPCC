#pragma once

#include "config.hpp"
#include "types.hpp"
#include "Params/params.hpp"

namespace mpcc
{
class Simulator {

public:
  Simulator(void config, Car car, Tire tire, void centerLine);
  State simTimeStep(const State& state, const Input& input, double ts) const;

private:
  State ode4(const State& state, const Input& input, double ts) const;
  State unwrapState(const State& state) const;
  State calculateDerivatives(const State& state, const Input& input) const;

private:
  Car car;
  Tire tire;
  Model model;
  Config config;
  void centerLine;
};
} // namespace mpcc
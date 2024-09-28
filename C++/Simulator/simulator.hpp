#pragma once

#include "config.hpp"
#include "types.hpp"
#include "Params/params.hpp"
#include "Models/models.hpp"
#include "Spline/arc_length_spline.hpp"

namespace mpcc
{
class Simulator {

public:
  Simulator(Car car, Tire tire, ArcLengthSpline centerLine);
  State simTimeStep(const State& state, const Input& input, double ts) const;

private:
  Car d_car;
  Tire d_tire;
  Model d_model;
  Models d_models;
  ArcLengthSpline d_centerLine;
};
} // namespace mpcc
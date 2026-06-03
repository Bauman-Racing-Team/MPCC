#pragma once

#include "config.hpp"
#include "types.hpp"
#include "Params/params.hpp"
#include "Models/models.hpp"
#include "Spline/arc_length_spline.hpp"

namespace mpcc
{
  class Simulator
  {

  public:
    Simulator(const PathToJson &jsonPath, const ArcLengthSpline &centerLine);
    State13 simTimeStep(const State13 &state, const Input &input, double ts) const;

  private:
    ArcLengthSpline d_centerLine;
    Models d_models;
  };
} // namespace mpcc
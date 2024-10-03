#include "simulator.hpp"
#include "Models/models.hpp"

namespace mpcc
{

  Simulator::Simulator(const PathToJson &jsonPath, const ArcLengthSpline &centerLine) : d_centerLine(centerLine), d_models(jsonPath)
  {
  }

  State Simulator::simTimeStep(const State &state, const Input &input, double ts) const
  {
    State nextState = state;
    int integrationSteps = static_cast<int>(ts / 0.001);

    double centerLineLength = d_centerLine.getLength();

    for (int i = 0; i < integrationSteps; ++i)
    {
      nextState = d_models.ode4(nextState, input, 0.001, std::bind(&Models::calculateSimpleDynamicModelDerivatives, &d_models, std::placeholders::_1, std::placeholders::_2));
      unwrapState(nextState, centerLineLength);
    }

    return nextState;
  }

} // namespace mpcc
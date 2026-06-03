#include "simulator.hpp"
#include "Models/models.hpp"

static constexpr double SIMULATION_DT = 0.001; // [s]

namespace mpcc
{

  Simulator::Simulator(const PathToJson &jsonPath, const ArcLengthSpline &centerLine) : d_centerLine(centerLine), d_models(jsonPath)
  {
  }

  State13 Simulator::simTimeStep(const State13 &state, const Input &input, double ts) const
  {
    State13 nextState = state;
    int integrationSteps = static_cast<int>(ts / SIMULATION_DT);

    double centerLineLength = d_centerLine.getLength();

    for (int i = 0; i < integrationSteps; ++i)
    {
      nextState = d_models.ode4(nextState, input, SIMULATION_DT, std::bind(&Models::calculateCombinedSlipDynamicModelDerivatives, &d_models, std::placeholders::_1, std::placeholders::_2));
      unwrapState(nextState, centerLineLength);
    }

    return nextState;
  }

} // namespace mpcc
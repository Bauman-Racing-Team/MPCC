#include "simulator.hpp"
#include "Models/models.hpp"

namespace mpcc
{

  Simulator::Simulator(Car car, Tire tire, ArcLengthSpline centerLine) : d_spline(spline), d_models(car, tire, centerLine)
  {
  }

  State simTimeStep(const State &state, const Input &input, double ts, std::function<State(State, Input)>) const
  {
    State nextState = state;
    int integrationSteps = static_cast<int>(ts / 0.001);

    double centerLineLength = d_centerLine.getLength();

    for (int i = 0; i < integrationSteps; ++i)
    {
      nextState = d_models.ode4(nextState, input, 0.001, std::bind(d_models.calculateCombinedSlipDynamicModelDerivatives(), d_models, std::placeholders::_1));
      unwrapState(nextState, centerLineLength);
    }

    return nextState;
  }

} // namespace mpcc
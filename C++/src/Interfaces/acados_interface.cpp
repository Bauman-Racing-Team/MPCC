#include "acados_interface.hpp"

namespace mpcc
{

AcadosInterface::AcadosInterface(
  const Bounds &bounds, const Config &config, const Cost &costs, double ts)
: d_bounds(bounds),
  d_config(config),
  d_costs(costs),
  d_dt(ts),
  d_newTimeSteps(new double[d_config.n]),
  d_minTime(std::numeric_limits<double>::max()),
  d_xTraj(new double[NX * (config.n + 1)]),
  d_uTraj(new double[NU * config.n])
{
}

AcadosInterface::~AcadosInterface()
{
  delete[] d_uTraj;
  delete[] d_xTraj;
  delete[] d_newTimeSteps;
  printf("Deleting Acados Interface.\n");
}

solverReturn AcadosInterface::solveMPC(
  std::vector<OptVariables> &initialGuess, AcadosParameters parameters)
{
  initMPC();
  setInitialValues(initialGuess);
  setParameters(parameters);
  return solve();
};
}  // namespace mpcc

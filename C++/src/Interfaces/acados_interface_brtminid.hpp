#pragma once

#include "acados_interface.hpp"

#include <vector>

namespace mpcc
{

class AcadosInterfaceBrtMiniD : public AcadosInterface
{
public:
  /**
   * @brief Constructor
   * @param bounds bounds
   * @param config config
   * @param costs costs
   * @param dt delta time
   *
   */
  AcadosInterfaceBrtMiniD(
    const Bounds &bounds, const Config &config, const Cost &costs, double dt);

protected:
  /**
   * @brief Initialize MPC Acados interface
   */
  void initMPC() final;

  /**
   * @brief Set initial values for solving the problem
   * @param initialGuess initial guess
   */
  void setInitialValues(std::vector<OptVariables> &initialGuess) final;

  /**
   * @brief Set parameters values
   * @param parameters acados parameters
   */
  void setParameters(AcadosParameters parameters) final;

  /**
   * @brief Solve MPC Problem
   * @return solver return
   */
  solverReturn solve() final;

  /**
   * @brief Generate solution
   */
  void generateSolution() final;

  /**
   * @brief Clear solver interface
   */
  void freeSolver() final;
};
}  // namespace mpcc

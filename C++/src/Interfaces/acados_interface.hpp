#pragma once

#include "config.hpp"
#include "Params/params.hpp"
#include "types.hpp"

#include <acados/utils/print.h>
#include <acados/utils/math.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_c/external_function_interface.h>
#include <blasfeo/include/blasfeo_d_aux_ext_dep.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>

namespace mpcc
{

class AcadosInterface
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
  AcadosInterface(const Bounds &bounds, const Config &config, const Cost &costs, double dt);

  /**
   * @brief Destructor
   */
  virtual ~AcadosInterface();

  /**
   * @brief Main function to solve MPC problem
   * @param initialGuess initial guess
   * @param parameters parameters values to solve the problem
   * @return solver return
   */
  solverReturn solveMPC(std::vector<OptVariables> &initialGuess, AcadosParameters parameters);

protected:
  /**
   * @brief Initialize MPC Acados interface
   */
  virtual void initMPC() = 0;

  /**
   * @brief Set initial values for solving the problem
   * @param initialGuess initial guess
   */
  virtual void setInitialValues(std::vector<OptVariables> &initialGuess) = 0;

  /**
   * @brief Set parameters values
   * @param parameters acados parameters
   */
  virtual void setParameters(AcadosParameters parameters) = 0;

  /**
   * @brief Solve MPC Problem
   * @return solver return
   */
  virtual solverReturn solve() = 0;

  /**
   * @brief Generate solution
   */
  virtual void generateSolution() = 0;

  /**
   * @brief Clear solver interface
   */
  virtual void freeSolver() = 0;

protected:
  const Bounds d_bounds;
  const Config d_config;
  const Cost d_costs;
  double d_dt;

  double *d_newTimeSteps;
  int d_status;

  void *d_nlpOpts;

  double d_minTime;
  double d_kktNormInf;
  int d_sqpIter;

  double *d_xTraj;
  double *d_uTraj;

  void *d_acadosOcpCapsule;

  void *d_nlpConfig;
  void *d_nlpDims;
  void *d_nlpIn;
  void *d_nlpOut;
  void *d_nlpSolver;
};
}  // namespace mpcc

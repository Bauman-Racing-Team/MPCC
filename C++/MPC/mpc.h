// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#ifndef MPCC_MPC_H
#define MPCC_MPC_H

#include "config.h"
#include "types.h"
#include "Params/params.h"
#include "Spline/arc_length_spline.h"
#include "Model/integrator.h"
#include "Cost/cost.h"
#include "Constraints/bounds.h"

// #include "Interfaces/solver_interface.h"
#include "Interfaces/acados_interface.h"

#include <array>
#include <memory>
#include <ctime>
#include <ratio>
#include <chrono>

namespace mpcc
{
struct Stage {
  CostMatrix cost_mat;

  Bounds_x u_bounds_x;
  Bounds_x l_bounds_x;

  Bounds_u u_bounds_u;
  Bounds_u l_bounds_u;

  Bounds_s u_bounds_s;
  Bounds_s l_bounds_s;

  // nx    -> number of states
  // nu    -> number of inputs
  // nbx   -> number of bounds on x
  // nbu   -> number of bounds on u
  // ng    -> number of polytopic constratins
  // ns   -> number of soft constraints
  int nx, nu, nbx, nbu, ng, ns;
};

struct MPCReturn {
  const Input u0;
  const std::array<OptVariables, N + 1> mpc_horizon;
  const double time_total;
  const int solverStatus;
};

class MPC
{
public:
  MPCReturn runMPC(State &x0);

  void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);

  MPC();
  MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path);

private:
  bool valid_initial_guess_;

  std::array<Stage, N + 1> stages_;
  std::array<Parameter, N + 1> parameter_;

  std::array<OptVariables, N + 1> initial_guess_;
  std::array<OptVariables, N + 1> temp_guess_;
  std::array<OptVariables, N + 1> optimal_solution_;

  void fillParametersVector();

  void setMPCProblem();

  void updateInitialGuess(const State &x0);

  void generateNewInitialGuess(const State &x0);

  void unwrapInitialGuess();

  int n_sqp_;
  double sqp_mixing_;
  int n_no_solves_sqp_;
  int n_no_solves_sqp_max_;
  int n_reset_;

  double bounds_x[2*NX];
  const double Ts_;

  Integrator integrator_;
  Cost cost_;
  ArcLengthSpline track_;

  Bounds bounds_;
  Param param_;
  CostParam cost_param_;

  std::unique_ptr<SolverInterface> solver_interface_;
};

}  // namespace mpcc

#endif  // MPCC_MPC_H

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

#include "config.hpp"
#include "types.hpp"
#include "Models/models.hpp"
#include "Params/params.hpp"
#include "Spline/arc_length_spline.hpp"

#include "Interfaces/acados_interface.hpp"

#include <array>
#include <memory>
#include <ctime>
#include <ratio>
#include <chrono>

namespace mpcc
{

struct MPCReturn {
  const Input u0;
  const std::array<OptVariables, N + 1> mpc_horizon;
  const double time_total;
  const int solverStatus;
};

class MPC
{
public:
  MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path);
  
  MPCReturn runMPC(const State &x0);

  void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);

  ArcLengthSpline getTrack() const;

private:
  void fillParametersVector();
  void setMPCProblem();
  void updateInitialGuess(const State &x0);
  void generateNewInitialGuess(const State &x0);
  void unwrapInitialGuess();

private:
  bool validInitialGuess;

  AcadosParameters parameter_;

  std::array<OptVariables, N + 1> initialGuess;
  std::array<OptVariables, N + 1> tempGuess;
  std::array<OptVariables, N + 1> optimalSolution;

  int nSqp;
  double sqpMixing;
  int nNoSolvesSqp;
  int nNoSolvesSqpMax;
  int nReset;

  double bounds_x[2 * NX];
  const double Ts_;

  ArcLengthSpline track_;

  Bounds bounds;
  Model model;
  Cost cost;
  Models models;

  std::unique_ptr<AcadosInterface> solverInterface;
};

}  // namespace mpcc

#endif  // MPCC_MPC_H

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
#include "mpc_track.hpp"
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
  const std::vector<OptVariables> mpc_horizon;
  const double time_total;
  const int solverStatus;
};

class MPC
{
public:
  MPC(const std::string &autonomousVehicle, const Bounds &bounds, const Config &config, const Cost &cost, const Car &car, const Tire &tire, double ts);
  
  MPCReturn runMPC(const State &x0);

  void genMpcTrack(const Eigen::VectorXd &x, const Eigen::VectorXd &y, 
    const Eigen::VectorXd &xOuter, const Eigen::VectorXd &yOuter, 
    const Eigen::VectorXd &xInner, const Eigen::VectorXd &yInner);

  ArcLengthSpline getMpcTrackCenterLine() const;
  MpcTrack getMpcTrack() const;

private:
  void fillParametersVector();
  void setMPCProblem();
  void updateInitialGuess(const State &x0);
  void generateNewInitialGuess(const State &x0);
  void unwrapInitialGuess();
  void calculateBordersInterpolations();
  std::pair<double, double> findRayBorderIntersection(double cx, double cy, double nx, double ny, const Eigen::VectorXd& bx, const Eigen::VectorXd& by);

private:
  const double d_ts;

  const Bounds d_bounds;
  const Config d_config;
  const Cost d_cost;

  const Car d_car;
  const Models d_models;

  bool d_validInitialGuess;

  AcadosParameters d_parameters;

  std::vector<OptVariables> d_initialGuess;

  MpcTrack d_mpcTrack;

  std::unique_ptr<AcadosInterface> d_solverInterfacePtr;
};

}  // namespace mpcc

#endif  // MPCC_MPC_H

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

#include "mpc.h"

namespace mpcc
{
MPC::MPC() : Ts_(0.05)
{
  // std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

MPC::MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path)
: Ts_(Ts),
  validInitialGuess(false),
  solverInterface(new AcadosInterface()),
  param_(Param(path.param_path)),
  costParam(CostParam(path.cost_path)),
  bounds_(BoundsParam(path.bounds_path)),
  track_(ArcLengthSpline(path))
{
  nSqp = n_sqp;
  sqpMixing = sqp_mixing;
  nNoSolvesSqp = 0;
  nReset = n_reset;
}

void MPC::fillParametersVector()
{
  for (int timeStep = 0; timeStep <= N; timeStep++) {
    parameter_[timeStep].setZero();
    Eigen::Vector2d trackPosI = track_.getPostion(initialGuess[timeStep].xk.s);
    Eigen::Vector2d trackDposI = track_.getDerivative(initialGuess[timeStep].xk.s);
    parameter_[timeStep].xTrack = trackPosI(0);
    parameter_[timeStep].yTrack = trackPosI(1);
    parameter_[timeStep].phiTrack = atan2(trackDposI(1), trackDposI(0));
    parameter_[timeStep].s0 = initialGuess[timeStep].xk.s;
    parameter_[timeStep].qC = costParam.qC;
    parameter_[timeStep].qL = costParam.qL;
    parameter_[timeStep].qVs = costParam.qVs;
    parameter_[timeStep].rdThrottle = costParam.rdThrottle;
    parameter_[timeStep].rdSteeringAngle = costParam.rdSteeringAngle;
    parameter_[timeStep].rdBrakes = costParam.rdBrakes;
    parameter_[timeStep].rdVs = costParam.rdVs;
    parameter_[timeStep].scQuadTrack = costParam.scQuadTrack;
    parameter_[timeStep].scQuadTire = costParam.scQuadTire;
    parameter_[timeStep].scQuadAlpha = costParam.scQuadAlpha;
    parameter_[timeStep].scQuadLonControl= costParam.scQuadLonControl;
    parameter_[timeStep].scLinTrack = costParam.scLinTrack;
    parameter_[timeStep].scLinTire = costParam.scLinTire;
    parameter_[timeStep].scLinAlpha = costParam.scLinAlpha;
    parameter_[timeStep].scLinLonControl = costParam.scLinLonControl;
  }
}

void MPC::updateInitialGuess(const State &x0)
{
  for (int i = 1; i < N; i++) initialGuess[i - 1].uk = initialGuess[i].uk;
  initialGuess[N - 1].uk = initialGuess[N - 2].uk;

  initialGuess[0].xk = x0;
  for (int i = 1; i < N; i++) initialGuess[i].xk = initialGuess[i + 1].xk;

  initialGuess[N].xk = integrator_.RK4(initialGuess[N - 1].xk, initialGuess[N - 1].uk, Ts_);
  initialGuess[N].uk.setZero();

  for (int i = 0; i < N + 1; i++) initialGuess[i].xk.vxVsNonZero(param_.vx_zero);
  unwrapInitialGuess();
}

void MPC::unwrapInitialGuess()
{
  double L = track_.getLength();
  for (int i = 1; i <= N; i++) {
    if ((initialGuess[i].xk.phi - initialGuess[i - 1].xk.phi) < -M_PI) {
      initialGuess[i].xk.phi += 2. * M_PI;
    }
    if ((initialGuess[i].xk.phi - initialGuess[i - 1].xk.phi) > M_PI) {
      initialGuess[i].xk.phi -= 2. * M_PI;
    }

    if ((initialGuess[i].xk.s - initialGuess[i - 1].xk.s) > L / 2.) {
      initialGuess[i].xk.s -= L;
    }
  }
}

void MPC::generateNewInitialGuess(const State &x0)
{
  initialGuess[0].xk = x0;
  initialGuess[0].xk.vxVsNonZero(param_.vx_zero);
  initialGuess[0].uk.setZero();

  for (int i = 1; i <= N; i++) {
    initialGuess[i].xk.setZero();
    initialGuess[i].uk.setZero();
    initialGuess[i].xk.vxVsNonZero(param_.vx_zero);

    initialGuess[i].xk.s = initialGuess[i - 1].xk.s + Ts_ * initialGuess[i - 1].xk.vs;
    Eigen::Vector2d trackPosI = track_.getPostion(initialGuess[i].xk.s);
    Eigen::Vector2d trackDposI = track_.getDerivative(initialGuess[i].xk.s);
    initialGuess[i].xk.X = trackPosI(0);
    initialGuess[i].xk.Y = trackPosI(1);
    initialGuess[i].xk.phi = atan2(trackDposI(1), trackDposI(0));
  }
  unwrapInitialGuess();
  validInitialGuess = true;
}

MPCReturn MPC::runMPC(State &x0)
{
  auto t1 = std::chrono::high_resolution_clock::now();
  int solver_status = -1;
  x0.s = track_.porjectOnSpline(x0);
  x0.unwrap(track_.getLength());

  nNoSolvesSqp = 0;
  nNoSolvesSqpMax = 0;

  while (nNoSolvesSqpMax < nSqp) {
    if (validInitialGuess)
      updateInitialGuess(x0);
    else
      generateNewInitialGuess(x0);

    fillParametersVector();

    solverReturn mpcSol = solverInterface->solveMPC(initialGuess, parameter_, bounds_);
    solver_status = mpcSol.status;

    if (solver_status == 0) {
      tempGuess = mpcSol.mpcHorizon;
      break;
    }
    if (solver_status == 2 || solver_status == 3) {
      tempGuess = mpcSol.mpcHorizon;
    }
    if (solver_status != 0) {
      nNoSolvesSqp++;
      if (nNoSolvesSqp >= nReset) {
        validInitialGuess = false;
        nNoSolvesSqp = 0;
      }
    }
    nNoSolvesSqpMax++;
  }

  if (nNoSolvesSqpMax < nSqp) initialGuess = tempGuess;

  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span =
    std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  double time_nmpc = time_span.count();

  return {initialGuess[0].uk, initialGuess, time_nmpc, solver_status};
}

void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y)
{
  track_.gen2DSpline(X, Y);
}

}  // namespace mpcc
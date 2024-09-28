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

#include "mpc.hpp"

namespace mpcc
{
MPC::MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path)
: Ts_(Ts),
  validInitialGuess(false),
  solverInterface(new AcadosInterface()),
  model(Model(path.param_path)),
  cost(Cost(path.cost_path)),
  bounds(Bounds(path.bounds_path)),
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
    parameter_ = AcadosParameters::Zero();
    Eigen::Vector2d trackPosI = track_.getPostion(initialGuess[timeStep].xk(s));
    Eigen::Vector2d trackDposI = track_.getDerivative(initialGuess[timeStep].xk(s));
    parameter_(xTrackP,timeStep) = trackPosI(0);
    parameter_(yTrackP,timeStep) = trackPosI(1);
    parameter_(yawTrackP,timeStep) = atan2(trackDposI(1), trackDposI(0));
    parameter_(s0P,timeStep) = initialGuess[timeStep].xk(s);
    parameter_(qCP,timeStep) = cost.qC;
    parameter_(qLP,timeStep) = cost.qL;
    parameter_(qVsP,timeStep) = cost.qVs;
    parameter_(rdThrottleP,timeStep) = cost.rdThrottle;
    parameter_(rdSteeringAngleP,timeStep) = cost.rdSteeringAngle;
    parameter_(rdBrakesP,timeStep) = cost.rdBrakes;
    parameter_(rdVsP,timeStep) = cost.rdVs;
    parameter_(scQuadAlphaFrontP,timeStep) = cost.scQuadAlpha;
    parameter_(scQuadAlphaRearP,timeStep) = cost.scQuadAlpha;
    parameter_(scQuadROutP,timeStep) = cost.scQuadTrack;
    parameter_(scQuadEllipseFrontP,timeStep) = cost.scQuadTire;
    parameter_(scQuadEllipseRearP,timeStep) = cost.scQuadTire;
    parameter_(scQuadLonControlP,timeStep) = cost.scQuadLonControl;
    parameter_(scLinAlphaFrontP,timeStep) = cost.scLinAlpha;
    parameter_(scLinAlphaRearP,timeStep) = cost.scLinAlpha;
    parameter_(scLinROutP,timeStep) = cost.scLinTrack;
    parameter_(scLinEllipseFrontP,timeStep) = cost.scLinTire;
    parameter_(scLinEllipseRearP,timeStep) = cost.scLinTire;    
    parameter_(scLinLonControlP,timeStep) = cost.scLinLonControl;
  }
}

void MPC::updateInitialGuess(const State &x0)
{
  for (int i = 1; i < N; i++) initialGuess[i - 1].uk = initialGuess[i].uk;
  initialGuess[N - 1].uk = initialGuess[N - 2].uk;

  initialGuess[0].xk = x0;
  for (int i = 1; i < N; i++) initialGuess[i].xk = initialGuess[i + 1].xk;

  initialGuess[N].xk = integrator_.RK4(initialGuess[N - 1].xk, initialGuess[N - 1].uk, Ts_);
  initialGuess[N].uk = Input::Zero();

  for (int i = 0; i < N + 1; i++) {
    vxVsNonZero(initialGuess[i].xk, model.vxMin);
  }
  unwrapInitialGuess();
}

void MPC::unwrapInitialGuess()
{
  double L = track_.getLength();
  for (int i = 1; i <= N; i++) {
    if ((initialGuess[i].xk(yaw) - initialGuess[i - 1].xk(yaw)) < -M_PI) {
      initialGuess[i].xk(yaw) += 2. * M_PI;
    }
    if ((initialGuess[i].xk(yaw) - initialGuess[i - 1].xk(yaw)) > M_PI) {
      initialGuess[i].xk(yaw) -= 2. * M_PI;
    }

    if ((initialGuess[i].xk(s) - initialGuess[i - 1].xk(s)) > L / 2.) {
      initialGuess[i].xk(s) -= L;
    }
  }
}

void MPC::generateNewInitialGuess(const State &x0)
{
  initialGuess[0].xk = x0;
  vxVsNonZero(initialGuess[0].xk, model.vxMin);
  initialGuess[0].uk.setZero();

  for (int i = 1; i <= N; i++) {
    initialGuess[i].xk = State::Zero();
    initialGuess[i].uk = Input::Zero();
    vxVsNonZero(initialGuess[i].xk, model.vxMin);

    initialGuess[i].xk(s) = initialGuess[i - 1].xk(s) + Ts_ * initialGuess[i - 1].xk(vs);
    Eigen::Vector2d trackPosI = track_.getPostion(initialGuess[i].xk(s));
    Eigen::Vector2d trackDposI = track_.getDerivative(initialGuess[i].xk(s));
    initialGuess[i].xk(X) = trackPosI(0);
    initialGuess[i].xk(Y) = trackPosI(1);
    initialGuess[i].xk(yaw) = atan2(trackDposI(1), trackDposI(0));
  }
  unwrapInitialGuess();
  validInitialGuess = true;
}

MPCReturn MPC::runMPC(State &x0)
{
  auto t1 = std::chrono::high_resolution_clock::now();
  int solver_status = -1;
  x0(s) = track_.porjectOnSpline(x0);
  unwrapState(x0, track_.getLength());

  nNoSolvesSqp = 0;
  nNoSolvesSqpMax = 0;

  while (nNoSolvesSqpMax < nSqp) {
    if (validInitialGuess)
      updateInitialGuess(x0);
    else
      generateNewInitialGuess(x0);

    fillParametersVector();

    solverReturn mpcSol = solverInterface->solveMPC(initialGuess, parameter_, bounds);
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
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

#include "Interfaces/acados_interface_brt8d.hpp"
#include "Interfaces/acados_interface_brt9d.hpp"
#include "Interfaces/acados_interface_brtminid.hpp"

namespace mpcc
{
  MPC::MPC(const std::string &autonomousVehicle, const Bounds &bounds, const Config &config, const Cost &cost, const Car &car, const Tire &tire, double ts)
      : d_ts(ts),
        d_bounds(bounds),
        d_config(config),
        d_cost(cost),
        d_car(car),
        d_models(car, tire),
        d_validInitialGuess(false),
        d_parameters(Eigen::MatrixXd(NP, config.n + 1)),
        d_initialGuess(config.n + 1),
        d_mpcTrack(config)
  {
    if (autonomousVehicle == "brt8d") {
    d_solverInterfacePtr = std::make_unique<AcadosInterfaceBrt8D>(bounds, config, cost, d_ts);
    } else if (autonomousVehicle == "brt9d") {
      d_solverInterfacePtr = std::make_unique<AcadosInterfaceBrt9D>(bounds, config, cost, d_ts);
    } else if (autonomousVehicle == "brtminid") {
      d_solverInterfacePtr = std::make_unique<AcadosInterfaceBrtMiniD>(bounds, config, cost, d_ts);
    } else {
      throw std::runtime_error(
        "Unknown autonomous vehicle name, can not create acados interface for it");
    }
  }

  void MPC::fillParametersVector()
  {
    d_parameters.setZero();
    for (int timeStep = 0; timeStep <= d_config.n; timeStep++)
    {
      double carX = d_initialGuess[timeStep].xk(xIdx);
      double carY = d_initialGuess[timeStep].xk(yIdx);
      double carS = d_initialGuess[timeStep].xk(sIdx);
            
      Eigen::Vector2d trackPosI = d_mpcTrack.getCenterLine().getPosition(carS);
      Eigen::Vector2d outerBorderPosI = d_mpcTrack.getOuterBorder().getPosition(carS);
      Eigen::Vector2d innerBorderPosI = d_mpcTrack.getInnerBorder().getPosition(carS);
      Eigen::Vector2d trackDposI = d_mpcTrack.getCenterLine().getDerivative(carS);

      d_parameters(xTrackP, timeStep) = trackPosI(0);
      d_parameters(yTrackP, timeStep) = trackPosI(1);
      d_parameters(yawTrackP, timeStep) = std::atan2(trackDposI(1), trackDposI(0));
      d_parameters(s0P, timeStep) = carS;
      d_parameters(vRefP, timeStep) = d_config.vRef;
      d_parameters(qCP, timeStep) = d_cost.qC;
      d_parameters(qLP, timeStep) = d_cost.qL;
      d_parameters(qVsP, timeStep) = d_cost.qVs;
      d_parameters(rdThrottleP, timeStep) = d_cost.rdThrottle;
      d_parameters(rdSteeringAngleP, timeStep) = d_cost.rdSteeringAngle;
      d_parameters(rdBrakesP, timeStep) = d_cost.rdBrakes;
      d_parameters(rdVsP, timeStep) = d_cost.rdVs;
      d_parameters(xOuterBorderP, timeStep) = outerBorderPosI(0);
      d_parameters(yOuterBorderP, timeStep) = outerBorderPosI(1);
      d_parameters(xInnerBorderP, timeStep) = innerBorderPosI(0);
      d_parameters(yInnerBorderP, timeStep) = innerBorderPosI(1);
    }
  }

  void MPC::updateInitialGuess(const State &x0)
  {
    for (int i = 1; i < d_config.n; i++)
      d_initialGuess[i - 1].uk = d_initialGuess[i].uk;
    d_initialGuess[d_config.n - 1].uk = d_initialGuess[d_config.n - 2].uk;

    d_initialGuess[0].xk = x0;
    for (int i = 1; i < d_config.n; i++)
      d_initialGuess[i].xk = d_initialGuess[i + 1].xk;

    d_initialGuess[d_config.n].xk = d_models.ode4(d_initialGuess[d_config.n - 1].xk, d_initialGuess[d_config.n - 1].uk, d_ts, std::bind(&Models::calculateSimpleCombinedModelDerivatives, &d_models, std::placeholders::_1, std::placeholders::_2));
    d_initialGuess[d_config.n].uk = Input::Zero();

    for (int i = 0; i < d_config.n + 1; i++)
    {
      vxVsNonZero(d_initialGuess[i].xk, d_config.vxMin);
    }
    unwrapInitialGuess();
  }

  void MPC::unwrapInitialGuess()
  {
    double centerLineLength = d_mpcTrack.getCenterLine().getLength();
    for (int i = 1; i <= d_config.n; i++)
    {
      if ((d_initialGuess[i].xk(yawIdx) - d_initialGuess[i - 1].xk(yawIdx)) < -M_PI)
      {
        d_initialGuess[i].xk(yawIdx) += 2. * M_PI;
      }
      else if ((d_initialGuess[i].xk(yawIdx) - d_initialGuess[i - 1].xk(yawIdx)) > M_PI)
      {
        d_initialGuess[i].xk(yawIdx) -= 2. * M_PI;
      }

      if ((d_initialGuess[i].xk(sIdx) - d_initialGuess[i - 1].xk(sIdx)) > centerLineLength / 2.)
      {
        d_initialGuess[i].xk(sIdx) -= centerLineLength;
      }
    }
  }

  void MPC::generateNewInitialGuess(const State &x0)
  {
    d_initialGuess[0].xk = x0;
    vxVsNonZero(d_initialGuess[0].xk, d_config.vxMin);
    d_initialGuess[0].uk.setZero();

    for (int i = 1; i <= d_config.n; i++)
    {
      d_initialGuess[i].xk = State::Zero();
      d_initialGuess[i].uk = Input::Zero();
      vxVsNonZero(d_initialGuess[i].xk, d_config.vxMin);

      d_initialGuess[i].xk(sIdx) = d_initialGuess[i - 1].xk(sIdx) + d_ts * d_initialGuess[i - 1].xk(vsIdx);
      Eigen::Vector2d trackPosI = d_mpcTrack.getCenterLine().getPosition(d_initialGuess[i].xk(sIdx));
      Eigen::Vector2d trackDposI = d_mpcTrack.getCenterLine().getDerivative(d_initialGuess[i].xk(sIdx));
      d_initialGuess[i].xk(xIdx) = trackPosI(0);
      d_initialGuess[i].xk(yIdx) = trackPosI(1);
      d_initialGuess[i].xk(yawIdx) = atan2(trackDposI(1), trackDposI(0));
    }
    unwrapInitialGuess();
    d_validInitialGuess = true;
  }

  MPCReturn MPC::runMPC(const State &x0)
  {
    State x = x0;
    auto t1 = std::chrono::high_resolution_clock::now();
    int solver_status = -1;
    x(sIdx) = d_mpcTrack.getCenterLine().porjectOnSpline(x);

    int nNoSolvesSqp = 0;
    int nNoSolvesSqpMax = 0;

    std::vector<OptVariables> tempGuess;

    while (nNoSolvesSqpMax < d_config.nSqp)
    {
      if (d_validInitialGuess)
        updateInitialGuess(x);
      else
        generateNewInitialGuess(x);

      fillParametersVector();

      solverReturn mpcSol = d_solverInterfacePtr->solveMPC(d_initialGuess, d_parameters);

      solver_status = mpcSol.status;

      if (solver_status != 0){
        throw std::runtime_error("solver_status: " + std::to_string(solver_status));
      }

      if (solver_status == 0)
      {
        tempGuess = mpcSol.mpcHorizon;
        break;
      }
      if (solver_status == 2 || solver_status == 3)
      {
        tempGuess = mpcSol.mpcHorizon;
      }
      if (solver_status != 0)
      {
        // std::cout << "Solved" << std::endl;
        nNoSolvesSqp++;
        if (nNoSolvesSqp >= d_config.nReset)
        {
          d_validInitialGuess = false;
          nNoSolvesSqp = 0;
        }
      }
      nNoSolvesSqpMax++;
    }

    if (nNoSolvesSqpMax < d_config.nSqp)
      d_initialGuess = tempGuess;

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    double time_nmpc = time_span.count();

    return {d_initialGuess[0].uk, d_initialGuess, time_nmpc, solver_status};
  }

  void MPC::genMpcTrack(const Eigen::VectorXd &x, const Eigen::VectorXd &y, 
                     const Eigen::VectorXd &xOuter, const Eigen::VectorXd &yOuter, 
                     const Eigen::VectorXd &xInner, const Eigen::VectorXd &yInner)
  {
    d_mpcTrack.generate(x, y, xOuter, yOuter, xInner, yInner);
  }

  MpcTrack MPC::getMpcTrack() const
  {
    return d_mpcTrack;
  }

  ArcLengthSpline MPC::getMpcTrackCenterLine() const 
  {
    return d_mpcTrack.getCenterLine();
  }



} // namespace mpcc
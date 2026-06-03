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
  MPC::MPC(const std::string &autonomousVehicle, const Bounds &bounds, const Config &config, const Cost &cost, const Car &car, const Tire &tire, double ts)
      : d_ts(ts),
        d_validInitialGuess(false),
        d_solverInterfacePtr(std::make_unique<AcadosInterface>()),
        d_bounds(bounds),
        d_config(config),
        d_cost(cost),
        d_car(car),
        d_models(car, tire)
  {
  }

  void MPC::fillParametersVector()
  {
    d_parameters.setZero();
    for (int timeStep = 0; timeStep <= N; timeStep++)
    {
      double carX = d_initialGuess[timeStep].xk(xIdx);
      double carY = d_initialGuess[timeStep].xk(yIdx);
      double carS = d_initialGuess[timeStep].xk(sIdx);
            
      Eigen::Vector2d trackPosI = d_centerLine.getPostion(carS);
      Eigen::Vector2d outerBorderPosI = d_outerBorder.getPostion(carS);
      Eigen::Vector2d innerBorderPosI = d_innerBorder.getPostion(carS);
      Eigen::Vector2d trackDposI = d_centerLine.getDerivative(carS);

      double minDistFromBorderToCarCenter = std::sqrt(std::min({std::pow((outerBorderPosI(0) - carX),2) + std::pow((outerBorderPosI(1) - carY),2),
                                                               std::pow((innerBorderPosI(0) - carX),2) + std::pow((innerBorderPosI(1) - carY),2),
                                                               std::pow((outerBorderPosI(0) - trackPosI(0)),2) + std::pow((innerBorderPosI(1) - trackPosI(1)),2),
                                                               std::pow((innerBorderPosI(0) - trackPosI(0)),2) + std::pow((innerBorderPosI(1) - trackPosI(1)),2)}));
     
      double sqareOfMinDistFromBorderToCar = std::pow((minDistFromBorderToCarCenter - d_config.safetyDistance - d_car.carW/2),2); 

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
      d_parameters(sqareOfMinDistFromBorderToCarP, timeStep) = sqareOfMinDistFromBorderToCar;
    }
  }

  void MPC::updateInitialGuess(const State &x0)
  {
    for (int i = 1; i < N; i++)
      d_initialGuess[i - 1].uk = d_initialGuess[i].uk;
    d_initialGuess[N - 1].uk = d_initialGuess[N - 2].uk;

    d_initialGuess[0].xk = x0;
    for (int i = 1; i < N; i++)
      d_initialGuess[i].xk = d_initialGuess[i + 1].xk;

    d_initialGuess[N].xk = d_models.ode4(d_initialGuess[N - 1].xk, d_initialGuess[N - 1].uk, d_ts, std::bind(&Models::calculateSimpleCombinedModelDerivatives, &d_models, std::placeholders::_1, std::placeholders::_2));
    d_initialGuess[N].uk = Input::Zero();

    for (int i = 0; i < N + 1; i++)
    {
      vxVsNonZero(d_initialGuess[i].xk, d_config.vxMin);
    }
    unwrapInitialGuess();
  }

  void MPC::unwrapInitialGuess()
  {
    double centerLineLength = d_centerLine.getLength();
    for (int i = 1; i <= N; i++)
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

    for (int i = 1; i <= N; i++)
    {
      d_initialGuess[i].xk = State::Zero();
      d_initialGuess[i].uk = Input::Zero();
      vxVsNonZero(d_initialGuess[i].xk, d_config.vxMin);

      d_initialGuess[i].xk(sIdx) = d_initialGuess[i - 1].xk(sIdx) + d_ts * d_initialGuess[i - 1].xk(vsIdx);
      Eigen::Vector2d trackPosI = d_centerLine.getPostion(d_initialGuess[i].xk(sIdx));
      Eigen::Vector2d trackDposI = d_centerLine.getDerivative(d_initialGuess[i].xk(sIdx));
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
    x(sIdx) = d_centerLine.porjectOnSpline(x);

    int nNoSolvesSqp = 0;
    int nNoSolvesSqpMax = 0;

    while (nNoSolvesSqpMax < d_config.nSqp)
    {
      if (d_validInitialGuess)
        updateInitialGuess(x);
      else
        generateNewInitialGuess(x);

      fillParametersVector();

      solverReturn mpcSol = d_solverInterfacePtr->solveMPC(d_initialGuess, d_parameters, d_bounds, d_cost);

      solver_status = mpcSol.status;

      if (solver_status == 0)
      {
        d_tempGuess = mpcSol.mpcHorizon;
        break;
      }
      if (solver_status == 2 || solver_status == 3)
      {
        d_tempGuess = mpcSol.mpcHorizon;
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
      d_initialGuess = d_tempGuess;

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    double time_nmpc = time_span.count();

    return {d_initialGuess[0].uk, d_initialGuess, time_nmpc, solver_status};
  }

  void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y, 
                     const Eigen::VectorXd &XOuter, const Eigen::VectorXd &YOuter, 
                     const Eigen::VectorXd &XInner, const Eigen::VectorXd &YInner)
  {
    d_centerLine.gen2DSpline(X, Y);
    d_outerBorder.gen2DSpline(XOuter, YOuter);
    d_innerBorder.gen2DSpline(XInner, YInner);
    
    calculateBordersInterpolations();
  }

  void MPC::calculateBordersInterpolations(){
    // Build perpendicular-offset border interpolations w.r.t. centerline normals
    auto centerLinePath = d_centerLine.getPath();
    int nPts = centerLinePath.n_points;

    Eigen::VectorXd outerPerpX(nPts);
    Eigen::VectorXd outerPerpY(nPts);
    Eigen::VectorXd innerPerpX(nPts);
    Eigen::VectorXd innerPerpY(nPts);

    outerPerpX.setZero();
    outerPerpY.setZero();
    innerPerpX.setZero();
    innerPerpY.setZero();

    // Use resampled border paths for nearest-neighbor search
    const PathData& outerBorderPath = d_outerBorder.getPath();
    const PathData& innerBorderPath = d_innerBorder.getPath();

    const Eigen::VectorXd& outerX = outerBorderPath.X;
    const Eigen::VectorXd& outerY = outerBorderPath.Y;
    const Eigen::VectorXd& innerX = innerBorderPath.X;
    const Eigen::VectorXd& innerY = innerBorderPath.Y;

    for (int i = 0; i < nPts; i++) {
        // Center point and tangent/normal
        double s = centerLinePath.s(i);
        Eigen::Vector2d centerPos = d_centerLine.getPostion(s);
        Eigen::Vector2d tangent = d_centerLine.getDerivative(s);
        
        double cx = centerPos(0);
        double cy = centerPos(1);
        double tx = tangent(0);
        double ty = tangent(1);
        
        double tnorm = std::sqrt(tx * tx + ty * ty);
        if (tnorm > 0) {
            tx /= tnorm;
            ty /= tnorm;
        }
        // Left-hand normal (rotate tangent 90 degrees counterclockwise)
        double nx = -ty;
        double ny = tx;

        // Nearest outer point
        Eigen::ArrayXd dxo = outerX.array() - cx;
        Eigen::ArrayXd dyo = outerY.array() - cy;
        Eigen::ArrayXd distSqOuter = dxo.square() + dyo.square();
        int idxO = 0;
        double minDistOuter = distSqOuter(0);
        for (int j = 1; j < outerX.size(); j++) {
            if (distSqOuter(j) < minDistOuter) {
                minDistOuter = distSqOuter(j);
                idxO = j;
            }
        }
        double pxo = outerX(idxO);
        double pyo = outerY(idxO);

        // Nearest inner point
        Eigen::ArrayXd dxi = innerX.array() - cx;
        Eigen::ArrayXd dyi = innerY.array() - cy;
        Eigen::ArrayXd distSqInner = dxi.square() + dyi.square();
        int idxI = 0;
        double minDistInner = distSqInner(0);
        for (int j = 1; j < innerX.size(); j++) {
            if (distSqInner(j) < minDistInner) {
                minDistInner = distSqInner(j);
                idxI = j;
            }
        }
        double pxi = innerX(idxI);
        double pyi = innerY(idxI);

        // Perpendicular distances along normal
        double wLeft = (pxo - cx) * nx + (pyo - cy) * ny;   // signed along +N
        double wRight = -((pxi - cx) * nx + (pyi - cy) * ny); // make positive to the right

        outerPerpX(i) = cx + wLeft * nx;
        outerPerpY(i) = cy + wLeft * ny;
        innerPerpX(i) = cx - wRight * nx;
        innerPerpY(i) = cy - wRight * ny;
    }

    d_outerBorder.updateSpline(outerPerpX, outerPerpY, centerLinePath.s);
    d_innerBorder.updateSpline(innerPerpX, innerPerpY, centerLinePath.s);
  }

  ArcLengthSpline MPC::getTrack() const
  {
    return d_centerLine;
  }

} // namespace mpcc
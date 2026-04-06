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
        model(Model(path.modelPath)),
        cost(Cost(path.costsPath)),
        d_car(Car(path.carPath)),
        models(Models(path)),
        bounds(Bounds(path.boundsPath)),
        centerLine_(ArcLengthSpline(path)),
        outerBorder_(ArcLengthSpline(path)),
        innerBorder_(ArcLengthSpline(path))
  {
    nSqp = n_sqp;
    sqpMixing = sqp_mixing;
    nNoSolvesSqp = 0;
    nReset = n_reset;
  }

  void MPC::fillParametersVector()
  {
    parameter_.setZero();
    for (int timeStep = 0; timeStep <= N; timeStep++)
    {
      double carX = initialGuess[timeStep].xk(xIdx);
      double carY = initialGuess[timeStep].xk(yIdx);
      double carS = initialGuess[timeStep].xk(sIdx);
            
      Eigen::Vector2d trackPosI = centerLine_.getPostion(carS);
      Eigen::Vector2d outerBorderPosI = outerBorder_.getPostion(carS);
      Eigen::Vector2d innerBorderPosI = innerBorder_.getPostion(carS);
      Eigen::Vector2d trackDposI = centerLine_.getDerivative(carS);

      double minDistFromBorderToCarCenter = std::sqrt(std::min({std::pow((outerBorderPosI(0) - carX),2) + std::pow((outerBorderPosI(1) - carY),2),
                                                               std::pow((innerBorderPosI(0) - carX),2) + std::pow((innerBorderPosI(1) - carY),2),
                                                               std::pow((outerBorderPosI(0) - trackPosI(0)),2) + std::pow((innerBorderPosI(1) - trackPosI(1)),2),
                                                               std::pow((innerBorderPosI(0) - trackPosI(0)),2) + std::pow((innerBorderPosI(1) - trackPosI(1)),2)}));
     
      double sqareOfMinDistFromBorderToCar = std::pow((minDistFromBorderToCarCenter - model.safetyDistance - d_car.carW/2),2); 

      parameter_(xTrackP, timeStep) = trackPosI(0);
      parameter_(yTrackP, timeStep) = trackPosI(1);
      parameter_(yawTrackP, timeStep) = std::atan2(trackDposI(1), trackDposI(0));
      parameter_(s0P, timeStep) = carS;
      parameter_(vRefP, timeStep) = model.vRef;
      parameter_(qCP, timeStep) = cost.qC;
      parameter_(qLP, timeStep) = cost.qL;
      parameter_(qVsP, timeStep) = cost.qVs;
      parameter_(rdThrottleP, timeStep) = cost.rdThrottle;
      parameter_(rdSteeringAngleP, timeStep) = cost.rdSteeringAngle;
      parameter_(rdBrakesP, timeStep) = cost.rdBrakes;
      parameter_(rdVsP, timeStep) = cost.rdVs;
      parameter_(sqareOfMinDistFromBorderToCarP, timeStep) = sqareOfMinDistFromBorderToCar;
    }
  }

  void MPC::updateInitialGuess(const State &x0)
  {
    for (int i = 1; i < N; i++)
      initialGuess[i - 1].uk = initialGuess[i].uk;
    initialGuess[N - 1].uk = initialGuess[N - 2].uk;

    initialGuess[0].xk = x0;
    for (int i = 1; i < N; i++)
      initialGuess[i].xk = initialGuess[i + 1].xk;

    initialGuess[N].xk = models.ode4(initialGuess[N - 1].xk, initialGuess[N - 1].uk, Ts_, std::bind(&Models::calculateSimpleCombinedModelDerivatives, &models, std::placeholders::_1, std::placeholders::_2));
    initialGuess[N].uk = Input::Zero();

    for (int i = 0; i < N + 1; i++)
    {
      vxVsNonZero(initialGuess[i].xk, model.vxMin);
    }
    unwrapInitialGuess();
  }

  void MPC::unwrapInitialGuess()
  {
    double centerLineLength = centerLine_.getLength();
    for (int i = 1; i <= N; i++)
    {
      if ((initialGuess[i].xk(yawIdx) - initialGuess[i - 1].xk(yawIdx)) < -M_PI)
      {
        initialGuess[i].xk(yawIdx) += 2. * M_PI;
      }
      else if ((initialGuess[i].xk(yawIdx) - initialGuess[i - 1].xk(yawIdx)) > M_PI)
      {
        initialGuess[i].xk(yawIdx) -= 2. * M_PI;
      }

      if ((initialGuess[i].xk(sIdx) - initialGuess[i - 1].xk(sIdx)) > centerLineLength / 2.)
      {
        initialGuess[i].xk(sIdx) -= centerLineLength;
      }
    }
  }

  void MPC::generateNewInitialGuess(const State &x0)
  {
    initialGuess[0].xk = x0;
    vxVsNonZero(initialGuess[0].xk, model.vxMin);
    initialGuess[0].uk.setZero();

    for (int i = 1; i <= N; i++)
    {
      initialGuess[i].xk = State::Zero();
      initialGuess[i].uk = Input::Zero();
      vxVsNonZero(initialGuess[i].xk, model.vxMin);

      initialGuess[i].xk(sIdx) = initialGuess[i - 1].xk(sIdx) + Ts_ * initialGuess[i - 1].xk(vsIdx);
      Eigen::Vector2d trackPosI = centerLine_.getPostion(initialGuess[i].xk(sIdx));
      Eigen::Vector2d trackDposI = centerLine_.getDerivative(initialGuess[i].xk(sIdx));
      initialGuess[i].xk(xIdx) = trackPosI(0);
      initialGuess[i].xk(yIdx) = trackPosI(1);
      initialGuess[i].xk(yawIdx) = atan2(trackDposI(1), trackDposI(0));
    }
    unwrapInitialGuess();
    validInitialGuess = true;
  }

  MPCReturn MPC::runMPC(const State &x0)
  {
    State x = x0;
    auto t1 = std::chrono::high_resolution_clock::now();
    int solver_status = -1;
    x(sIdx) = centerLine_.porjectOnSpline(x);

    nNoSolvesSqp = 0;
    nNoSolvesSqpMax = 0;

    while (nNoSolvesSqpMax < nSqp)
    {
      if (validInitialGuess)
        updateInitialGuess(x);
      else
        generateNewInitialGuess(x);

      fillParametersVector();

      solverReturn mpcSol = solverInterface->solveMPC(initialGuess, parameter_, bounds, cost);

      solver_status = mpcSol.status;

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
        if (nNoSolvesSqp >= nReset)
        {
          validInitialGuess = false;
          nNoSolvesSqp = 0;
        }
      }
      nNoSolvesSqpMax++;
    }

    if (nNoSolvesSqpMax < nSqp)
      initialGuess = tempGuess;

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    double time_nmpc = time_span.count();

    return {initialGuess[0].uk, initialGuess, time_nmpc, solver_status};
  }

  void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y, 
                     const Eigen::VectorXd &XOuter, const Eigen::VectorXd &YOuter, 
                     const Eigen::VectorXd &XInner, const Eigen::VectorXd &YInner)
  {
    centerLine_.gen2DSpline(X, Y);
    outerBorder_.gen2DSpline(XOuter, YOuter);
    innerBorder_.gen2DSpline(XInner, YInner);
    
    calculateBordersInterpolations();
  }

  void MPC::calculateBordersInterpolations(){
    // Build perpendicular-offset border interpolations w.r.t. centerline normals
    auto centerLinePath = centerLine_.getPath();
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
    const PathData& outerBorderPath = outerBorder_.getPath();
    const PathData& innerBorderPath = innerBorder_.getPath();

    const Eigen::VectorXd& outerX = outerBorderPath.X;
    const Eigen::VectorXd& outerY = outerBorderPath.Y;
    const Eigen::VectorXd& innerX = innerBorderPath.X;
    const Eigen::VectorXd& innerY = innerBorderPath.Y;

    for (int i = 0; i < nPts; i++) {
        // Center point and tangent/normal
        double s = centerLinePath.s(i);
        Eigen::Vector2d centerPos = centerLine_.getPostion(s);
        Eigen::Vector2d tangent = centerLine_.getDerivative(s);
        
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

    outerBorder_.updateSpline(outerPerpX, outerPerpY, centerLinePath.s);
    innerBorder_.updateSpline(innerPerpX, innerPerpY, centerLinePath.s);
  }

  ArcLengthSpline MPC::getTrack() const
  {
    return centerLine_;
  }

} // namespace mpcc
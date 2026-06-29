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
        d_centerLine(config),
        d_outerBorder(config),
        d_innerBorder(config)
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
            
      Eigen::Vector2d trackPosI = d_centerLine.getPosition(carS);
      Eigen::Vector2d outerBorderPosI = d_outerBorder.getPosition(carS);
      Eigen::Vector2d innerBorderPosI = d_innerBorder.getPosition(carS);
      Eigen::Vector2d trackDposI = d_centerLine.getDerivative(carS);

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
    double centerLineLength = d_centerLine.getLength();
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
      Eigen::Vector2d trackPosI = d_centerLine.getPosition(d_initialGuess[i].xk(sIdx));
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

  void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y, 
                     const Eigen::VectorXd &XOuter, const Eigen::VectorXd &YOuter, 
                     const Eigen::VectorXd &XInner, const Eigen::VectorXd &YInner)
  {
    d_centerLine.gen2DSpline(X, Y);
    d_outerBorder.setPath(XOuter, YOuter);
    d_innerBorder.setPath(XInner, YInner);
    
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

    for (int i = 0; i < nPts; i++)
    {
      double s = centerLinePath.s(i);
      Eigen::Vector2d centerPos = d_centerLine.getPosition(s);
      Eigen::Vector2d tangent = d_centerLine.getDerivative(s);

      double cx = centerPos(0);
      double cy = centerPos(1);
      double tx = tangent(0);
      double ty = tangent(1);
      
      double tnorm = std::hypot(tx,ty);
      if(tnorm > 0)
      {
        tx = tx/tnorm; 
        ty = ty/tnorm;
      }

      double nx = -ty; 
      double ny = tx; // Левая нормаль

      /* --- РАСЧЕТ ДЛЯ ВНЕШНЕЙ ГРАНИЦЫ (Вдоль +N) */
      auto [xO, yO] = findRayBorderIntersection(cx, cy, nx, ny, outerX, outerY);
      outerPerpX(i) = xO;
      outerPerpY(i) = yO;

      /* --- РАСЧЕТ ДЛЯ ВНУТРЕННЕЙ ГРАНИЦЫ (Вдоль -N) */
      auto [xI, yI] = findRayBorderIntersection(cx, cy, -nx, -ny, innerX, innerY);
      innerPerpX(i) = xI;
      innerPerpY(i) = yI;
    }

    d_outerBorder.genBorderInterpolation(outerPerpX, outerPerpY, centerLinePath.s);
    d_innerBorder.genBorderInterpolation(innerPerpX, innerPerpY, centerLinePath.s);
  }

  std::pair<double, double> MPC::findRayBorderIntersection(double cx, double cy, double nx, double ny, const Eigen::VectorXd& bx, const Eigen::VectorXd& by){        
    double bestX = cx; 
    double bestY = cy;
    double minT = 1e9; // Ищем минимальный положительный шаг вдоль луча
    
    double nSegs = bx.rows() - 1;
    
    for(int j = 0; j < nSegs; j++){
      // Вершины текущего сегмента границы
      double x1 = bx(j);   
      double y1 = by(j);
      double x2 = bx(j+1); 
      double y2 = by(j+1);
      
      // Вектор сегмента границы
      double dx = x2 - x1;
      double dy = y2 - y1;
      
      // Знаменатель (определитель матрицы системы)
      double det = nx * dy - ny * dx;
      
      // Если det == 0, луч и сегмент параллельны
      if(std::abs(det) < 1e-9)
      {
        continue;
      }

      // Решение системы линейных уравнений по правилу Крамера
      // t - расстояние вдоль луча нормали
      // u - положение точки на отрезке границы (от 0 до 1)
      double t = ((x1 - cx) * dy - (y1 - cy) * dx) / det;
      double u = ((x1 - cx) * ny - (y1 - cy) * nx) / det;
      
      // Проверяем, что пересечение впереди по лучу и попадает на отрезок
      if(t >= 0 && u >= 0 && u <= 1){
        if(t < minT)
        {
          minT = t;
          bestX = cx + t * nx;
          bestY = cy + t * ny;
        }
      }
    }
    return {bestX, bestY};
  }

  ArcLengthSpline MPC::getTrack() const
  {
    return d_centerLine;
  }

} // namespace mpcc
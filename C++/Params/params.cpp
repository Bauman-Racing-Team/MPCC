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

#include "params.h"
namespace mpcc
{

Param::Param()
{
  std::cout << "Default initialization of model params" << std::endl;
}

Param::Param(std::string file)
{
  /////////////////////////////////////////////////////
  // Loading Model and Constraint Parameters //////////
  /////////////////////////////////////////////////////
  // std::cout << "model" << std::endl;

  std::ifstream iModel(file);
  json jsonModel;
  iModel >> jsonModel;
  // Model Parameters

  car_l = jsonModel["car_l"];
  car_w = jsonModel["car_w"];

  max_dist_proj = jsonModel["maxDistProj"];

  vxMin = jsonModel["vxMin"];
}

CostParam::CostParam()
{
  std::cout << "Default initialization of cost" << std::endl;
}

CostParam::CostParam(std::string file)
{
  /////////////////////////////////////////////////////
  // Loading Cost Parameters //////////////////////////
  /////////////////////////////////////////////////////
  // std::cout << "cost" << std::endl;

  std::ifstream iCost(file);
  json jsonCost;
  iCost >> jsonCost;

  qC = jsonCost["qC"];
  qL = jsonCost["qL"];
  qVs = jsonCost["qVs"];

  qMu = jsonCost["qMu"];

  qR = jsonCost["qR"];

  qBeta = jsonCost["qBeta"];
  betaKinCost = jsonCost["betaKin"];

  rdThrottle = jsonCost["rdThrottle"];
  rdSteeringAngle = jsonCost["rdSteeringAngle"];
  rdBrakes = jsonCost["rdBrakes"];
  rdVs = jsonCost["rdVs"];

  scQuadTrack = jsonCost["sc_quad_track"];
  scQuadTire = jsonCost["sc_quad_tire"];
  scQuadAlpha = jsonCost["sc_quad_alpha"];
  scQuadLonControl = jsonCost["sc_quad_lon_control"];

  scLinTrack = jsonCost["sc_lin_track"];
  scLinTire = jsonCost["sc_lin_tire"];
  scLinAlpha = jsonCost["sc_lin_alpha"];
  scLinLonControl = jsonCost["sc_lin_lon_control"];

  std::cout << "rdThrottle: " << rdThrottle << std::endl;
  std::cout << "rdSteeringAngle: " << rdSteeringAngle << std::endl;
  std::cout << "rdBrakes: " << rdBrakes << std::endl;
  std::cout << "rdVs: " << rdVs << std::endl;
}

BoundsParam::BoundsParam()
{
  std::cout << "Default initialization of bounds" << std::endl;
}

BoundsParam::BoundsParam(std::string file)
{
  /////////////////////////////////////////////////////
  // Loading Cost Parameters //////////////////////////
  /////////////////////////////////////////////////////
  // std::cout << "bounds" << std::endl;

  std::ifstream iBounds(file);
  json jsonBounds;
  iBounds >> jsonBounds;

  lower_state_bounds.xL = jsonBounds["xL"];
  lower_state_bounds.yL = jsonBounds["yL"];
  lower_state_bounds.phiL = jsonBounds["phiL"];
  lower_state_bounds.vxL = jsonBounds["vxL"];
  lower_state_bounds.vyL = jsonBounds["vyL"];
  lower_state_bounds.rL = jsonBounds["rL"];
  lower_state_bounds.sL = jsonBounds["sL"];
  lower_state_bounds.throttleL = jsonBounds["throttleL"];
  lower_state_bounds.steeringAngleL = jsonBounds["steeringAngleL"];
  lower_state_bounds.brakesL = jsonBounds["brakesL"];
  lower_state_bounds.vsL = jsonBounds["vsL"];

  upper_state_bounds.xU = jsonBounds["xU"];
  upper_state_bounds.yU = jsonBounds["yU"];
  upper_state_bounds.phiU = jsonBounds["phiU"];
  upper_state_bounds.vxU = jsonBounds["vxU"];
  upper_state_bounds.vyU = jsonBounds["vyU"];
  upper_state_bounds.rU = jsonBounds["rU"];
  upper_state_bounds.sU = jsonBounds["sU"];
  upper_state_bounds.throttleU = jsonBounds["throttleU"];
  upper_state_bounds.steeringAngleU = jsonBounds["steeringAngleU"];
  upper_state_bounds.brakesU = jsonBounds["brakesU"];
  upper_state_bounds.vsU = jsonBounds["vsU"];

  lower_input_bounds.dThrottleL= jsonBounds["dThrottleL"];
  lower_input_bounds.dBrakesL= jsonBounds["dBrakesL"];
  lower_input_bounds.dSteeringAngleL = jsonBounds["dSteeringAngleL"];
  lower_input_bounds.dVsL = jsonBounds["dVsL"];

  upper_input_bounds.dThrottleU= jsonBounds["dThrottleU"];
  upper_input_bounds.dSteeringAngleU= jsonBounds["dSteeringAngleU"];
  upper_input_bounds.dBrakesU = jsonBounds["dBrakesU"];
  upper_input_bounds.dVsU = jsonBounds["dVsU"];

  lower_const_bounds.maxAlphaL = jsonBounds["maxAlphaL"];
  lower_const_bounds.rOutL = jsonBounds["rOutL"];
  lower_const_bounds.ellipseL = jsonBounds["ellipseL"];
  lower_const_bounds.lonControlL = jsonBounds["lonControlL"];

  upper_const_bounds.maxAlphaU = jsonBounds["maxAlphaU"];
  upper_const_bounds.rOutU = jsonBounds["rOutU"];
  upper_const_bounds.ellipseU = jsonBounds["ellipseU"];
  upper_const_bounds.lonControlU = jsonBounds["lonControlU"];
}
}  // namespace mpcc

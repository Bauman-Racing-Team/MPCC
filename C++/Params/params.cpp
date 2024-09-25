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
  Cm1 = jsonModel["Cm1"];
  Cm2 = jsonModel["Cm2"];

  Cr0 = jsonModel["Cr0"];
  Cr2 = jsonModel["Cr2"];
  CBf = jsonModel["CBf"];
  CBr = jsonModel["CBr"];

  Cl = jsonModel["Cl"];
  rho = jsonModel["rho"];
  S = jsonModel["S"];
  aero_split_front = jsonModel["aero_split_front"];

  Br = jsonModel["Br"];
  Cr = jsonModel["Cr"];
  Dr = jsonModel["Dr"];

  Bf = jsonModel["Bf"];
  Cf = jsonModel["Cf"];
  Df = jsonModel["Df"];

  m = jsonModel["m"];
  Iz = jsonModel["Iz"];
  lf = jsonModel["lf"];
  lr = jsonModel["lr"];

  car_l = jsonModel["car_l"];
  car_w = jsonModel["car_w"];

  g = jsonModel["g"];
  // Constraint Parameters
  r_in = jsonModel["R_in"];
  r_out = jsonModel["R_out"];

  max_dist_proj = jsonModel["max_dist_proj"];

  e_long = jsonModel["E_long"];
  e_eps = jsonModel["E_eps"];

  max_alpha = jsonModel["maxAlpha"];
  // initial warm start and trust region (model dependent)
  initial_velocity = jsonModel["initial_velocity"];
  s_trust_region = jsonModel["s_trust_region"];

  vx_zero = jsonModel["vx_zero"];
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

  rThrottle = jsonCost["rThrottle"];
  rSteeringAngle = jsonCost["rSteeringAngle"];
  rBrakes = jsonCost["rBrakes"];
  rVs = jsonCost["rVs"];

  rdThrottle = jsonCost["rdThrottle"];
  rdSteeringAngle = jsonCost["rdSteeringAngle"];
  rdBrakes = jsonCost["rdBrakes"];
  rdVs = jsonCost["rdVs"];

  qCNmult = jsonCost["qCNmult"];
  qRNmult = jsonCost["qRNmult"];

  scQuadTrack = jsonCost["sc_quad_track"];
  scQuadTire = jsonCost["sc_quad_tire"];
  scQuadAlpha = jsonCost["sc_quad_alpha"];
  scQuadLonControl = jsonCost["sc_quad_lon_control"];

  scLinTrack = jsonCost["sc_lin_track"];
  scLinTire = jsonCost["sc_lin_tire"];
  scLinAlpha = jsonCost["sc_lin_alpha"];
  scLinLonControl = jsonCost["sc_lin_lon_control"];
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
  lower_state_bounds.rL = jsonBounds["vyL"];
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

  upper_const_bounds.maxAlphaU = jsonBounds["maxAlphaU"];
  upper_const_bounds.rOutU = jsonBounds["rOutU"];
  upper_const_bounds.ellipseU = jsonBounds["ellipseU"];
}
}  // namespace mpcc

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

#include "params.hpp"
namespace mpcc
{

Model::Model(const std::string& file)
{
  /////////////////////////////////////////////////////
  // Loading Model Parameters //////////
  /////////////////////////////////////////////////////

  std::ifstream iModel(file);
  json jsonModel;
  iModel >> jsonModel;
  // Model Parameters

  maxDistProj = jsonModel["maxDistProj"];
  vxMin = jsonModel["vxMin"];
}

Cost::Cost(const std::string& file)
{
  /////////////////////////////////////////////////////
  // Loading Cost Parameters //////////////////////////
  /////////////////////////////////////////////////////

  std::ifstream iCost(file);
  json jsonCost;
  iCost >> jsonCost;

  qC = jsonCost["qC"];
  qL = jsonCost["qL"];
  qVs = jsonCost["qVs"];

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
}

Bounds::Bounds(const std::string& file)
{
  /////////////////////////////////////////////////////
  // Loading Cost Parameters //////////////////////////
  /////////////////////////////////////////////////////

  std::ifstream iBounds(file);
  json jsonBounds;
  iBounds >> jsonBounds;

  stateLowerBounds(xL) = jsonBounds["xL"];
  stateLowerBounds(yL) = jsonBounds["yL"];
  stateLowerBounds(yawL) = jsonBounds["yawL"];
  stateLowerBounds(vxL) = jsonBounds["vxL"];
  stateLowerBounds(vyL) = jsonBounds["vyL"];
  stateLowerBounds(rL) = jsonBounds["rL"];
  stateLowerBounds(sL) = jsonBounds["sL"];
  stateLowerBounds(throttleL) = jsonBounds["throttleL"];
  stateLowerBounds(steeringAngleL) = jsonBounds["steeringAngleL"];
  stateLowerBounds(brakesL) = jsonBounds["brakesL"];
  stateLowerBounds(vsL) = jsonBounds["vsL"];

  stateUpperBounds(xU) = jsonBounds["xU"];
  stateUpperBounds(yU) = jsonBounds["yU"];
  stateUpperBounds(yawU) = jsonBounds["yawU"];
  stateUpperBounds(vxU) = jsonBounds["vxU"];
  stateUpperBounds(vyU) = jsonBounds["vyU"];
  stateUpperBounds(rU) = jsonBounds["rU"];
  stateUpperBounds(sU) = jsonBounds["sU"];
  stateUpperBounds(throttleU) = jsonBounds["throttleU"];
  stateUpperBounds(steeringAngleU) = jsonBounds["steeringAngleU"];
  stateUpperBounds(brakesU) = jsonBounds["brakesU"];
  stateUpperBounds(vsU) = jsonBounds["vsU"];

  inputLowerBounds(dThrottleL) = jsonBounds["dThrottleL"];
  inputLowerBounds(dSteeringAngleL) = jsonBounds["dSteeringAngleL"];
  inputLowerBounds(dBrakesL) = jsonBounds["dBrakesL"];
  inputLowerBounds(dVsL) = jsonBounds["dVsL"];

  inputUpperBounds(dThrottleU) = jsonBounds["dThrottleU"];
  inputUpperBounds(dSteeringAngleU) = jsonBounds["dSteeringAngleU"];
  inputUpperBounds(dBrakesU) = jsonBounds["dBrakesU"];
  inputUpperBounds(dVsU) = jsonBounds["dVsU"];

  constraintsLowerBounds(maxAlphaFrontL) = jsonBounds["maxAlphaFrontL"];
  constraintsLowerBounds(maxAlphaRearL) = jsonBounds["maxAlphaRearL"];
  constraintsLowerBounds(rOutL) = jsonBounds["rOutL"];
  constraintsLowerBounds(ellipseFrontL) = jsonBounds["ellipseFrontL"];
  constraintsLowerBounds(ellipseRearL) = jsonBounds["ellipseRearL"];
  constraintsLowerBounds(lonControlL) = jsonBounds["lonControlL"];

  constraintsUpperBounds(maxAlphaFrontU) = jsonBounds["maxAlphaFrontU"];
  constraintsUpperBounds(maxAlphaRearU) = jsonBounds["maxAlphaRearU"];
  constraintsUpperBounds(rOutU) = jsonBounds["rOutU"];
  constraintsUpperBounds(ellipseFrontU) = jsonBounds["ellipseFrontU"];
  constraintsUpperBounds(ellipseRearU) = jsonBounds["ellipseRearU"];
  constraintsUpperBounds(lonControlU) = jsonBounds["lonControlU"];
}

Car::Car(const std::string& file){
  std::ifstream iCar(file);
  json jsonCar;
  iCar >> jsonCar;

  m = jsonCar["m"];
  g = jsonCar["g"];
  iz = jsonCar["iz"];

  lf = jsonCar["lf"];
  lr = jsonCar["lr"];

  cl = jsonCar["cl"];
  cd = jsonCar["cd"];
  cm = jsonCar["cm"];

  brakesRatio = jsonCar["brakesRatio"];
  gearRatio = jsonCar["gearRatio"];
  pMax = jsonCar["pMax"];

  carL = jsonCar["carL"];
  carW = jsonCar["carW"];
}

Tire::Tire(const std::string& file){
  std::ifstream iTire(file);
  json jsonTire;
  iTire >> jsonTire;

  fzNominal = jsonTire["fzNominal"];
  R = jsonTire["R"];
  I = jsonTire["I"];

  muyFz = jsonTire["muyFz"]; 
  muxFz = jsonTire["muxFz"];

  LFZO = jsonTire["LFZO"];				
  LCX = jsonTire["LCX"];					
  LMUX = jsonTire["LMUX"];					
  LEX = jsonTire["LEX"];				
  LKX	= jsonTire["LKX"];				
  LHX	= jsonTire["LHX"];			
  LVX = jsonTire["LVX"];					
  LGAX = jsonTire["LGAX"];					
  LCY	= jsonTire["LCY"];			
  LMUY = jsonTire["LMUY"];				
  LEY = jsonTire["LEY"];					
  LKY	= jsonTire["LKY"];			
  LHY	= jsonTire["LHY"];			
  LVY	= jsonTire["LVY"];				
  LGAY = jsonTire["LGAY"];				
  LTR = jsonTire["LTR"];				
  LRES = jsonTire["LRES"];					
  LGAZ = jsonTire["LGAZ"];					
  LXAL = jsonTire["LXAL"];				
  LYKA = jsonTire["LYKA"];					
  LVYKA = jsonTire["LVYKA"];					
  LS = jsonTire["LS"];				
  LSGKP = jsonTire["LSGKP"];				
  LSGAL = jsonTire["LSGAL"];					
  LGYR = jsonTire["LGYR"];					
  LMX = jsonTire["LMX"]; 				
  LVMX = jsonTire["LVMX"];				
  LMY = jsonTire["LMY"];				

  PCX1 = jsonTire["PCX1"];
  PDX1 = jsonTire["PDX1"];			
  PDX2 = jsonTire["PDX2"];	
  PDX3 = jsonTire["PDX3"];			
  PEX1 = jsonTire["PEX1"];	
  PEX2 = jsonTire["PEX2"];
  PEX3 = jsonTire["PEX3"];	
  PEX4 = jsonTire["PEX4"];	
  PKX1 = jsonTire["PKX1"];		
  PKX2 = jsonTire["PKX2"];	
  PKX3 = jsonTire["PKX3"];		
  PHX1 = jsonTire["PHX1"];		
  PHX2 = jsonTire["PHX2"];			
  PVX1 = jsonTire["PVX1"];			
  PVX2 = jsonTire["PVX2"];			
  RBX1 = jsonTire["RBX1"];			
  RBX2 = jsonTire["RBX2"];		
  RCX1 = jsonTire["RCX1"];			
  REX1 = jsonTire["REX1"];		
  REX2 = jsonTire["REX2"];			
  RHX1 = jsonTire["RHX1"];			
  PTX1 = jsonTire["PTX1"];			
  PTX2 = jsonTire["PTX2"];			
  PTX3 = jsonTire["PTX3"];		

  PCY1 = jsonTire["PCY1"];			
  PDY1 = jsonTire["PDY1"];			
  PDY2 = jsonTire["PDY2"];		
  PDY3 = jsonTire["PDY3"];			
  PEY1 = jsonTire["PEY1"];		
  PEY2 = jsonTire["PEY2"];		
  PEY3 = jsonTire["PEY3"];		
  PEY4 = jsonTire["PEY4"];			
  PKY1 = jsonTire["PKY1"];		
  PKY2 = jsonTire["PKY2"];		
  PKY3 = jsonTire["PKY3"];			
  PHY1 = jsonTire["PHY1"];			
  PHY2 = jsonTire["PHY2"];		
  PHY3 = jsonTire["PHY3"];		
  PVY1 = jsonTire["PVY1"];		
  PVY2 = jsonTire["PVY2"];			
  PVY3 = jsonTire["PVY3"];	
  PVY4 = jsonTire["PVY4"];	
  RBY1 = jsonTire["RBY1"];		
  RBY2 = jsonTire["RBY2"];		
  RBY3 = jsonTire["RBY3"];
  RCY1 = jsonTire["RCY1"];		
  REY1 = jsonTire["REY1"];	
  REY2 = jsonTire["REY2"];		
  RHY1 = jsonTire["RHY1"];		
  RHY2 = jsonTire["RHY2"];		
  RVY1 = jsonTire["RVY1"];			
  RVY2 = jsonTire["RVY2"];
  RVY3 = jsonTire["RVY3"];			
  RVY4 = jsonTire["RVY4"];		
  RVY5 = jsonTire["RVY5"];		
  RVY6 = jsonTire["RVY6"];			
  PTY1 = jsonTire["PTY1"];			
  PTY2 = jsonTire["PTY2"];		

  QSY1 = jsonTire["QSY1"];	
  QSY2 = jsonTire["QSY2"];		
  QSY3 = jsonTire["QSY3"];	
  QSY4 = jsonTire["QSY4"];		

  Cy = jsonTire["Cy"];
}
}  // namespace mpcc


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

Config::Config(const std::string& file)
{
  /////////////////////////////////////////////////////
  // Loading Config //////////
  /////////////////////////////////////////////////////

  std::ifstream iConfig(file);
  json jsonConfig;
  iConfig >> jsonConfig;

  n = jsonConfig["n"];
  maxDistProj = jsonConfig["maxDistProj"];
  safetyDistance = jsonConfig["safetyDistance"];
  vxMin = jsonConfig["vxMin"];
  vRef = jsonConfig["vRef"];
  nSqp = jsonConfig["nSqp"];
  nReset = jsonConfig["nReset"];
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

  scQuadAlphaFront = jsonCost["scQuadAlphaFront"];
  scQuadAlphaRear = jsonCost["scQuadAlphaRear"];
  scQuadROut = jsonCost["scQuadROut"];
  scQuadEllipseFront = jsonCost["scQuadEllipseFront"];
  scQuadEllipseRear = jsonCost["scQuadEllipseRear"];
  scQuadLonControl = jsonCost["scQuadLonControl"];

  scLinAlphaFront = jsonCost["scLinAlphaFront"];
  scLinAlphaRear = jsonCost["scLinAlphaRear"];
  scLinROut = jsonCost["scLinROut"];
  scLinEllipseFront = jsonCost["scLinEllipseFront"];
  scLinEllipseRear = jsonCost["scLinEllipseRear"];
  scLinLonControl = jsonCost["scLinLonControl"];
}

Bounds::Bounds(const std::string& file)
{
  /////////////////////////////////////////////////////
  // Loading Cost Parameters //////////////////////////
  /////////////////////////////////////////////////////

  std::ifstream iBounds(file);
  json jsonBounds;
  iBounds >> jsonBounds;

  auto lowerStateBounds = jsonBounds["lowerStateBounds"];
  auto upperStateBounds = jsonBounds["upperStateBounds"];
  auto lowerInputBounds = jsonBounds["lowerInputBounds"];
  auto upperInputBounds = jsonBounds["upperInputBounds"];
  auto lowerConstraintBounds = jsonBounds["lowerConstraintBounds"];
  auto upperConstraintBounds = jsonBounds["upperConstraintBounds"];

  stateLowerBounds(xL) = lowerStateBounds["xL"];
  stateLowerBounds(yL) = lowerStateBounds["yL"];
  stateLowerBounds(yawL) = lowerStateBounds["yawL"];
  stateLowerBounds(vxL) = lowerStateBounds["vxL"];
  stateLowerBounds(vyL) = lowerStateBounds["vyL"];
  stateLowerBounds(rL) = lowerStateBounds["rL"];
  stateLowerBounds(sL) = lowerStateBounds["sL"];
  stateLowerBounds(throttleL) = lowerStateBounds["throttleL"];
  stateLowerBounds(steeringAngleL) = lowerStateBounds["steeringAngleL"];
  stateLowerBounds(brakesL) = lowerStateBounds["brakesL"];
  stateLowerBounds(vsL) = lowerStateBounds["vsL"];

  stateUpperBounds(xU) = upperStateBounds["xU"];
  stateUpperBounds(yU) = upperStateBounds["yU"];
  stateUpperBounds(yawU) = upperStateBounds["yawU"];
  stateUpperBounds(vxU) = upperStateBounds["vxU"];
  stateUpperBounds(vyU) = upperStateBounds["vyU"];
  stateUpperBounds(rU) = upperStateBounds["rU"];
  stateUpperBounds(sU) = upperStateBounds["sU"];
  stateUpperBounds(throttleU) = upperStateBounds["throttleU"];
  stateUpperBounds(steeringAngleU) = upperStateBounds["steeringAngleU"];
  stateUpperBounds(brakesU) = upperStateBounds["brakesU"];
  stateUpperBounds(vsU) = upperStateBounds["vsU"];

  inputLowerBounds(dThrottleL) = lowerInputBounds["dThrottleL"];
  inputLowerBounds(dBrakesL) = lowerInputBounds["dBrakesL"];
  inputLowerBounds(dSteeringAngleL) = lowerInputBounds["dSteeringAngleL"];
  inputLowerBounds(dVsL) = lowerInputBounds["dVsL"];

  inputUpperBounds(dThrottleU) = upperInputBounds["dThrottleU"];
  inputUpperBounds(dBrakesU) = upperInputBounds["dBrakesU"];
  inputUpperBounds(dSteeringAngleU) = upperInputBounds["dSteeringAngleU"];
  inputUpperBounds(dVsU) = upperInputBounds["dVsU"];

  constraintsLowerBounds(maxAlphaFrontL) = lowerConstraintBounds["maxAlphaFrontL"];
  constraintsLowerBounds(maxAlphaRearL) = lowerConstraintBounds["maxAlphaRearL"];
  constraintsLowerBounds(trackOuterBorderL) = lowerConstraintBounds["trackOuterBorderL"];
  constraintsLowerBounds(trackInnerBorderL) = lowerConstraintBounds["trackInnerBorderL"];
  constraintsLowerBounds(ellipseFrontL) = lowerConstraintBounds["ellipseFrontL"];
  constraintsLowerBounds(ellipseRearL) = lowerConstraintBounds["ellipseRearL"];
  constraintsLowerBounds(lonControlL) = lowerConstraintBounds["lonControlL"];

  constraintsUpperBounds(maxAlphaFrontU) = upperConstraintBounds["maxAlphaFrontU"];
  constraintsUpperBounds(maxAlphaRearU) = upperConstraintBounds["maxAlphaRearU"];
  constraintsUpperBounds(trackOuterBorderU) = upperConstraintBounds["trackOuterBorderU"];
  constraintsUpperBounds(trackInnerBorderU) = upperConstraintBounds["trackInnerBorderU"];
  constraintsUpperBounds(ellipseFrontU) = upperConstraintBounds["ellipseFrontU"];
  constraintsUpperBounds(ellipseRearU) = upperConstraintBounds["ellipseRearU"];
  constraintsUpperBounds(lonControlU) = upperConstraintBounds["lonControlU"];
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


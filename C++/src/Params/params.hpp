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

#ifndef MPCC_PARAMS_H
#define MPCC_PARAMS_H

#include <vector>
#include "nlohmann/json.hpp"
#include "config.hpp"
#include "types.hpp"

namespace mpcc{
//used namespace
using json = nlohmann::json;

class Model{
public:
    Model(const std::string& file);
    
    double maxDistProj;
    double vxMin;
};

class Cost{
public:
    Cost(const std::string& file);

    double qC;
    double qL;
    double qVs;

    double rdThrottle;
    double rdSteeringAngle;
    double rdBrakes;
    double rdVs;

    double scQuadTrack;
    double scQuadTire;
    double scQuadAlpha;
    double scQuadLonControl;

    double scLinTrack;
    double scLinTire;
    double scLinAlpha;
    double scLinLonControl;
};

using StateLowerBound = Eigen::Matrix<double, NX, 1>;
using StateUpperBound = Eigen::Matrix<double, NX, 1>;

using InputLowerBounds = Eigen::Matrix<double, NU, 1>;
using InputUpperBounds = Eigen::Matrix<double, NU, 1>;

using ConstreaintsLowerBound = Eigen::Matrix<double , NS, 1>;
using ConstreaintsUpperBound = Eigen::Matrix<double , NS, 1>;

enum {
    xL = 0,
    yL = 1,
    yawL = 2,
    vxL = 3,
    vyL = 4,
    rL = 5,
    sL = 6,
    throttleL = 7,
    steeringAngleL = 8,
    brakesL = 9,
    vsL = 10
};

enum {
    xU = 0,
    yU = 1,
    yawU = 2,
    vxU = 3,
    vyU = 4,
    rU = 5,
    sU = 6,
    throttleU = 7,
    steeringAngleU = 8,
    brakesU = 9,
    vsU = 10
};

enum {
    dThrottleL = 0,
    dBrakesL = 1,
    dSteeringAngleL = 2,
    dVsL = 3,
};

enum {
    dThrottleU = 0,
    dBrakesU = 1,
    dSteeringAngleU = 2,
    dVsU = 3,
};

enum {
    maxAlphaFrontL = 0,
    maxAlphaRearL = 1,
    rOutL = 2,
    ellipseFrontL = 3,
    ellipseRearL = 4,
    lonControlL = 5,
};

enum {
    maxAlphaFrontU = 0,
    maxAlphaRearU = 1,
    rOutU = 2,
    ellipseFrontU = 3,
    ellipseRearU = 4,
    lonControlU = 5,
};

class Bounds{
public:
    Bounds(const std::string& file);

    StateLowerBound stateLowerBounds;
    StateUpperBound stateUpperBounds;

    InputLowerBounds inputLowerBounds;
    InputUpperBounds inputUpperBounds;

    ConstreaintsLowerBound constraintsLowerBounds;
    ConstreaintsUpperBound constraintsUpperBounds;
};

class Car {
public:
    Car(const std::string& file);

public:
    double m; // [kg] car mass
    double g; // [m/s^2] free fall acc
    double iz; // [kg * m^2] car inertia

    double lf; // [m] distance from COG to front axle
    double lr; // [m] distance from COG to rear axle

    double cl; // [-] aero lift coeff
    double cd; // [-] aero drag coeff
    double cm; // [-] aero moment

    double brakesRatio; // [-] brakes ratio: front/rear
    double gearRatio; // [-] main gear ratio
    double pMax; // [W] max power

    double carL; // [m] car length
    double carW; // [m] car width
};

class Tire {
public:
    Tire(const std::string& file);

public:
  double fzNominal; // [N] nominal tire load
  double R; // [m] unloaded tire radius
  double I; // [kg * m^2] tire inertia

  double muyFz; // [-] coeff of friction
  double muxFz; // [-] coeff of friction

  double LFZO;					
  double LCX;					
  double LMUX;					
  double LEX;				
  double LKX;					
  double LHX;				
  double LVX; 					
  double LGAX; 					
  double LCY;				
  double LMUY; 					
  double LEY; 					
  double LKY; 				
  double LHY; 					
  double LVY;					
  double LGAY;				
  double LTR; 				
  double LRES; 					
  double LGAZ; 					
  double LXAL; 				
  double LYKA; 					
  double LVYKA; 					
  double LS;				
  double LSGKP; 				
  double LSGAL; 					
  double LGYR; 					
  double LMX; 				
  double LVMX; 				
  double LMY; 				

  double PCX1;
  double PDX1;			
  double PDX2;		
  double PDX3;			
  double PEX1;		
  double PEX2;		
  double PEX3;			
  double PEX4;			
  double PKX1;		
  double PKX2;	
  double PKX3;		
  double PHX1;		
  double PHX2;			
  double PVX1;			
  double PVX2;			
  double RBX1;			
  double RBX2;		
  double RCX1;			
  double REX1;		
  double REX2;			
  double RHX1;			
  double PTX1;			
  double PTX2;			
  double PTX3;		

  double PCY1;			
  double PDY1;			
  double PDY2;		
  double PDY3;			
  double PEY1;		
  double PEY2;		
  double PEY3;		
  double PEY4;			
  double PKY1;		
  double PKY2;		
  double PKY3;			
  double PHY1;			
  double PHY2;		
  double PHY3;		
  double PVY1;		
  double PVY2;			
  double PVY3;	
  double PVY4;	
  double RBY1;		
  double RBY2;		
  double RBY3;
  double RCY1;		
  double REY1;	
  double REY2;		
  double RHY1;		
  double RHY2;		
  double RVY1;			
  double RVY2;	
  double RVY3;			
  double RVY4;		
  double RVY5;		
  double RVY6;			
  double PTY1;			
  double PTY2;			

  double QSY1;		
  double QSY2;			
  double QSY3;	
  double QSY4;		

  double Cy;
};

using AcadosParameters = Eigen::Matrix<double, 23, N + 1>;

enum {
    xTrackP = 0,
    yTrackP = 1,
    yawTrackP = 2,
    s0P = 3,
    qCP = 4,
    qLP = 5,
    qVsP = 6,
    rdThrottleP = 7,
    rdSteeringAngleP = 8,
    rdBrakesP = 9,
    rdVsP = 10,
    scQuadAlphaFrontP = 11,
    scQuadAlphaRearP = 12,
    scQuadROutP = 13,
    scQuadEllipseFrontP = 14,
    scQuadEllipseRearP = 15,
    scQuadLonControlP = 16,
    scLinAlphaFrontP = 17,
    scLinAlphaRearP = 18,
    scLinROutP = 19,
    scLinEllipseFrontP = 20,
    scLinEllipseRearP = 21,
    scLinLonControlP = 22
};


}
#endif //MPCC_PARAMS_H

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
#include <nlohmann/json.hpp>
#include "config.h"
#include "types.h"

namespace mpcc{
//used namespace
using json = nlohmann::json;

class Param{
public:
    double Cm1;
    double Cm2;
    double Cr0;
    double Cr2;
    double CBf;
    double CBr;
    double Cl;
    double rho;
    double S;
    double aero_split_front;

    double Br;
    double Cr;
    double Dr;

    double Bf;
    double Cf;
    double Df;

    double m;
    double Iz;
    double lf;
    double lr;

    double car_l;
    double car_w;

    double g;

    double r_in;
    double r_out;

    double max_dist_proj;

    double e_long;
    double e_eps;

    double max_alpha;

    double initial_velocity;
    double s_trust_region;

    double vx_zero;

    Param();
    Param(std::string file);

};

class CostParam{
public:
    double qC;
    double qL;
    double qVs;

    double qMu;

    double qR;

    double qBeta;
    int betaKinCost;

    double rThrottle;
    double rSteeringAngle;
    double rBrakes;
    double rVs;

    double rdThrottle;
    double rdSteeringAngle;
    double rdBrakes;
    double rdVs;

    double qCNmult;
    double qRNmult;

    double scQuadTrack;
    double scQuadTire;
    double scQuadAlpha;
    double scQuadControl;

    double scLinTrack;
    double scLinTire;
    double scLinAlpha;
    double scLinControl;

    CostParam();
    CostParam(std::string file);

};

class BoundsParam{
public:
    struct LowerStateBounds{
        double xL;
        double yL;
        double phiL;
        double vxL;
        double vyL;
        double rL;
        double sL;
        double throttleL;
        double steeringAngleL;
        double brakesL;
        double vsL;
    };
    struct UpperStateBounds{
        double xU;
        double yU;
        double phiU;
        double vxU;
        double vyU;
        double rU;
        double sU;
        double throttleU;
        double steeringAngleU;
        double brakesU;
        double vsU;
    };
    struct LowerInputBounds{
        double dThrottleL;
        double dBrakesL;
        double dSteeringAngleL;
        double dVsL;
    };
    struct UpperInputBounds{
        double dThrottleU;
        double dSteeringAngleU;
        double dBrakesU;
        double dVsU;
    };
    struct LowerConstBounds{
        double maxAlphaL;
        double rOutL;
        double ellipseL;
    };
    struct UpperConstBounds{
        double maxAlphaU;
        double rOutU;
        double ellipseU;
    };

    LowerStateBounds lower_state_bounds;
    UpperStateBounds upper_state_bounds;

    LowerInputBounds lower_input_bounds;
    UpperInputBounds upper_input_bounds;

    LowerConstBounds lower_const_bounds;
    UpperConstBounds upper_const_bounds;

    BoundsParam();
    BoundsParam(std::string file);

};
}
#endif //MPCC_PARAMS_H

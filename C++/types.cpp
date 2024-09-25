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

#include "types.h"
namespace mpcc{

StateVector stateToVector(const State &x)
{
    StateVector xk;
    xk(siIndex.X) = x.X;
    xk(siIndex.Y) = x.Y;
    xk(siIndex.phi) = x.phi;
    xk(siIndex.vx) = x.vx;
    xk(siIndex.vy) = x.vy;
    xk(siIndex.r) = x.r;
    xk(siIndex.s) = x.s;
    xk(siIndex.throttle) = x.throttle;
    xk(siIndex.steeringAngle) = x.steeringAngle;
    xk(siIndex.brakes) = x.brakes;
    xk(siIndex.vs) = x.vs;
    return xk;
}

InputVector inputToVector(const Input &u)
{
    InputVector uk = {u.dThrottle,u.dBrakes,u.dSteeringAngle,u.dVs};
    return uk;
}

State vectorToState(const StateVector &xk)
{
    State x;
    x.X     = xk(siIndex.X);
    x.Y     = xk(siIndex.Y);
    x.phi   = xk(siIndex.phi);
    x.vx    = xk(siIndex.vx);
    x.vy    = xk(siIndex.vy);
    x.r     = xk(siIndex.r);
    x.s     = xk(siIndex.s);
    x.throttle     = xk(siIndex.throttle);
    x.steeringAngle = xk(siIndex.steeringAngle);
    x.brakes     = xk(siIndex.brakes);
    x.vs    = xk(siIndex.vs);

    return x;
}

Input vectorToInput(const InputVector &uk)
{
    Input u;
    u.dThrottle     = uk(siIndex.dThrottle);
    u.dSteeringAngle = uk(siIndex.dSteeringAngle);
    u.dBrakes     = uk(siIndex.dBrakes);
    u.dVs    = uk(siIndex.dVs);

    return u;
}

State arrayToState(double *xk)
{
    State x;
    x.X     = xk[siIndex.X];
    x.Y     = xk[siIndex.Y];
    x.phi   = xk[siIndex.phi];
    x.vx    = xk[siIndex.vx];
    x.vy    = xk[siIndex.vy];
    x.r     = xk[siIndex.r];
    x.s     = xk[siIndex.s];
    x.throttle     = xk[siIndex.throttle];
    x.steeringAngle = xk[siIndex.steeringAngle];
    x.brakes     = xk[siIndex.brakes];
    x.vs    = xk[siIndex.vs];

    return x;
}

Input arrayToInput(double *uk)
{
    Input u;
    u.dThrottle     = uk[siIndex.dThrottle];
    u.dSteeringAngle = uk[siIndex.dSteeringAngle];
    u.dBrakes     = uk[siIndex.dBrakes];
    u.dVs    = uk[siIndex.dVs];

    return u;
}

std::vector<double> stateInputToVector(const State x,const Input u)
{

    StateVector xv = stateToVector(x);
    InputVector uv = inputToVector(u);

    Eigen::Vector<double,NX+NU> z;
    z << xv,uv;
    std::vector<double> zv(z.data(), z.data() + z.size());

    return zv;
}

}
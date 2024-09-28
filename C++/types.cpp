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

#include "types.hpp"
namespace mpcc{
State arrayToState(double *xk)
{
    State x;
    x(X)     = xk[X];
    x(Y)     = xk[Y];
    x(yaw)   = xk[yaw];
    x(vx)    = xk[vx];
    x(vy)    = xk[vy];
    x(r)     = xk[r];
    x(s)     = xk[s];
    x(throttle) = xk[throttle];
    x(steeringAngle) = xk[steeringAngle];
    x(brakes)  = xk[brakes];
    x(vs)  = xk[vs];

    return x;
}

Input arrayToInput(double *uk)
{
    Input u;
    u(dThrottle)     = uk[dThrottle];
    u(dSteeringAngle) = uk[dSteeringAngle];
    u(dBrakes)     = uk[dBrakes];
    u(dVs)    = uk[dVs];

    return u;
}

std::vector<double> stateInputToVector(const State& x,const Input& u)
{
    Eigen::Vector<double,NX+NU> z;
    z << x,u;
    std::vector<double> zv(z.data(), z.data() + z.size());

    return zv;
}

}
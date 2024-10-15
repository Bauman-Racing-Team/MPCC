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

void vxVsNonZero(State& state, double vxMin)
{
  if (state(vxIdx) < vxMin) {
    state(vxIdx) = vxMin;
  } 
  if (state(vsIdx) < vxMin) {
    state(vsIdx) = vxMin;
  } 
}
State arrayToState(double *xk)
{
    State x;
    x(xIdx)     = xk[xIdx];
    x(yIdx)     = xk[yIdx];
    x(yawIdx)   = xk[yawIdx];
    x(vxIdx)    = xk[vxIdx];
    x(vyIdx)    = xk[vyIdx];
    x(rIdx)     = xk[rIdx];
    x(sIdx)     = xk[sIdx];
    x(throttleIdx) = xk[throttleIdx];
    x(steeringAngleIdx) = xk[steeringAngleIdx];
    x(brakesIdx)  = xk[brakesIdx];
    x(vsIdx)  = xk[vsIdx];

    return x;
}

Input arrayToInput(double *uk)
{
    Input u;
    u(dThrottleIdx)     = uk[dThrottleIdx];
    u(dSteeringAngleIdx) = uk[dSteeringAngleIdx];
    u(dBrakesIdx)     = uk[dBrakesIdx];
    u(dVsIdx)    = uk[dVsIdx];

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
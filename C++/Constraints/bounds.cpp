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

#include "bounds.h"
namespace mpcc{
Bounds::Bounds()
{
    // std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Bounds::Bounds(BoundsParam bounds_param) 
{
    l_bounds_x_(siIndex.X) = bounds_param.lower_state_bounds.xL;
    l_bounds_x_(siIndex.Y) = bounds_param.lower_state_bounds.yL;
    l_bounds_x_(siIndex.phi) = bounds_param.lower_state_bounds.phiL;
    l_bounds_x_(siIndex.vx) = bounds_param.lower_state_bounds.vxL;
    l_bounds_x_(siIndex.vy) = bounds_param.lower_state_bounds.vyL;
    l_bounds_x_(siIndex.r) = bounds_param.lower_state_bounds.rL;
    l_bounds_x_(siIndex.s) = bounds_param.lower_state_bounds.sL;
    l_bounds_x_(siIndex.throttle) = bounds_param.lower_state_bounds.throttleL;
    l_bounds_x_(siIndex.steeringAngle) = bounds_param.lower_state_bounds.steeringAngleL;
    l_bounds_x_(siIndex.brakes) = bounds_param.lower_state_bounds.brakesL;
    l_bounds_x_(siIndex.vs) = bounds_param.lower_state_bounds.vsL;

    u_bounds_x_(siIndex.X) = bounds_param.upper_state_bounds.xU;
    u_bounds_x_(siIndex.Y) = bounds_param.upper_state_bounds.yU;
    u_bounds_x_(siIndex.phi) = bounds_param.upper_state_bounds.phiU;
    u_bounds_x_(siIndex.vx) = bounds_param.upper_state_bounds.vxU;
    u_bounds_x_(siIndex.vy) = bounds_param.upper_state_bounds.vyU;
    u_bounds_x_(siIndex.r) = bounds_param.upper_state_bounds.rU;
    u_bounds_x_(siIndex.s) = bounds_param.upper_state_bounds.sU;
    u_bounds_x_(siIndex.throttle) = bounds_param.upper_state_bounds.throttleU;
    u_bounds_x_(siIndex.steeringAngle) = bounds_param.upper_state_bounds.steeringAngleU;
    u_bounds_x_(siIndex.brakes) = bounds_param.upper_state_bounds.brakesU;
    u_bounds_x_(siIndex.vs) = bounds_param.upper_state_bounds.vsU;

    l_bounds_u_(siIndex.dThrottle) = bounds_param.lower_input_bounds.dThrottleL;
    l_bounds_u_(siIndex.dSteeringAngle) = bounds_param.lower_input_bounds.dSteeringAngleL;
    l_bounds_u_(siIndex.dBrakes) = bounds_param.lower_input_bounds.dBrakesL;
    l_bounds_u_(siIndex.dVs) = bounds_param.lower_input_bounds.dVsL;

    u_bounds_u_(siIndex.dThrottle) = bounds_param.upper_input_bounds.dThrottleU;
    u_bounds_u_(siIndex.dSteeringAngle) = bounds_param.upper_input_bounds.dSteeringAngleU;
    u_bounds_u_(siIndex.dBrakes) = bounds_param.upper_input_bounds.dBrakesU;
    u_bounds_u_(siIndex.dVs) = bounds_param.upper_input_bounds.dVsU;

    l_bounds_s_(siIndex.conTireF) = bounds_param.lower_const_bounds.maxAlphaL;
    l_bounds_s_(siIndex.conTireR) = bounds_param.lower_const_bounds.maxAlphaL;
    l_bounds_s_(siIndex.conTrack) = bounds_param.lower_const_bounds.rOutL;
    l_bounds_s_(siIndex.conElipF) = bounds_param.lower_const_bounds.ellipseL;
    l_bounds_s_(siIndex.conElipR) = bounds_param.lower_const_bounds.ellipseL;
    l_bounds_s_(siIndex.conElipR) = bounds_param.lower_const_bounds.lonControlL;

    u_bounds_s_(siIndex.conTireF) = bounds_param.upper_const_bounds.maxAlphaU;
    u_bounds_s_(siIndex.conTireR) = bounds_param.upper_const_bounds.maxAlphaU;
    u_bounds_s_(siIndex.conTrack) = bounds_param.upper_const_bounds.rOutU;
    u_bounds_s_(siIndex.conElipF) = bounds_param.upper_const_bounds.ellipseU;
    u_bounds_s_(siIndex.conElipR) = bounds_param.upper_const_bounds.ellipseU;
    u_bounds_s_(siIndex.conElipR) = bounds_param.upper_const_bounds.lonControlU;

    std::cout << "bounds initialized" << std::endl;
}

Bounds_x Bounds::getBoundsLX() const
{
    return  l_bounds_x_;
}

Bounds_x Bounds::getBoundsUX() const
{
    return  u_bounds_x_;
}

Bounds_u Bounds::getBoundsLU() const
{
    return  l_bounds_u_;
}

Bounds_u Bounds::getBoundsUU() const
{
    return  u_bounds_u_;
}

Bounds_s Bounds::getBoundsLS() const
{
    return  l_bounds_s_;
}

Bounds_s Bounds::getBoundsUS() const{
    return  u_bounds_s_;
}
}
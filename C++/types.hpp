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

#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include "config.hpp"
#include <vector>

namespace mpcc
{

enum {
  X = 0,
  Y = 1,
  yaw = 2,
  vx = 3,
  vy = 4,
  r = 5,
  s = 6,
  throttle = 7,
  steeringAngle = 8,
  brakes = 9,
  vs = 10,
  omegaf = 11,
  omegar = 12
};

using State = Eigen::Matrix<double, NX, 1>;
void unwrapState(State& state, double trackLength)
{
  if (state(yaw) > M_PI) {
    state(yaw) -= 2. * M_PI;
  }
  if (state(yaw) < -M_PI) {
    state(yaw) += 2. * M_PI;
  }
  if (s > trackLength) {
    state(s) -= trackLength;
  } 
  if (s < 0.) {
    state(s) += trackLength;
  } 
}

void vxVsNonZero(State& state, double vxMin)
{
  if (state(vx) < vxMin) {
    state(vx) = vxMin;
  } 
  if (state(vs) < vxMin) {
    state(vs) = vxMin;
  } 
}


enum {
  dThrottle = 0,
  dSteeringAngle = 1,
  dBrakes = 2,
  dVs = 3,
};

using Input = Eigen::Matrix<double, NU, 1>;
struct OptVariables {
  State xk;
  Input uk;
};

struct solverReturn {
  std::array<OptVariables, N + 1> mpcHorizon;
  int status;
};

struct PathToJson {
  const std::string param_path;
  const std::string cost_path;
  const std::string bounds_path;
  const std::string track_path;
  const std::string adcodegen_path;
};

State arrayToState(double *xk);
Input arrayToInput(double *uk);

std::vector<double> stateInputToVector(const State x, const Input u);
}  // namespace mpcc
#endif  // MPCC_TYPES_H

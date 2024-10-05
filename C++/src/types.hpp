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
  xIdx = 0,
  yIdx = 1,
  yawIdx = 2,
  vxIdx = 3,
  vyIdx = 4,
  rIdx = 5,
  sIdx = 6,
  throttleIdx = 7,
  steeringAngleIdx = 8,
  brakesIdx = 9,
  vsIdx = 10,
  omegafIdx = 11,
  omegarIdx = 12
};

using State = Eigen::Matrix<double, NX, 1>;
using State13 = Eigen::Matrix<double, 13, 1>;

template<typename S> 
void unwrapState(S& state, double trackLength){
  if (state(yawIdx) > M_PI) {
    state(yawIdx) -= 2. * M_PI;
  }
  if (state(yawIdx) < -M_PI) {
    state(yawIdx) += 2. * M_PI;
  }
  if (state(sIdx) > trackLength) {
    state(sIdx) -= trackLength;
  } 
  if (state(sIdx) < 0.) {
    state(sIdx) += trackLength;
  }
}

void vxVsNonZero(State& state, double vxMin);

enum {
  dThrottleIdx = 0,
  dSteeringAngleIdx = 1,
  dBrakesIdx = 2,
  dVsIdx = 3,
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
  std::string modelPath;
  std::string costsPath;
  std::string boundsPath;
  std::string trackPath;
  std::string carPath;
  std::string tirePath;
};

State arrayToState(double *xk);
Input arrayToInput(double *uk);

std::vector<double> stateInputToVector(const State x, const Input u);
}  // namespace mpcc
#endif  // MPCC_TYPES_H

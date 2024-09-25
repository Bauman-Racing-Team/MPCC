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

#include "integrator.h"
namespace mpcc
{
Integrator::Integrator()
{
  // std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

State Integrator::RK4(const State &x, const Input &u, double ts)
{
  State xNext = Simulate(x, u, ts);
  return xNext;
}

State Integrator::simTimeStep(const State &x, const Input &u, const double ts)
{
  // integrate time step
  State xNext = x;
  const int integrationSteps = (int)(ts / FineTimeStep);
  if (ts / FineTimeStep != integrationSteps) {
    std::cout << "Warning" << std::endl;
  }
  for (int i = 0; i < integrationSteps; i++) xNext = RK4(xNext, u, FineTimeStep);
  return xNext;
}
}  // namespace mpcc
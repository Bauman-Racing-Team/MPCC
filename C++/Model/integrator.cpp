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
  std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Integrator::Integrator(double Ts, const PathToJson &path) {}

State Integrator::RK4(const State &x, const Input &u, double ts)
{
  State xNext = Simulate(x, u, ts);
  return xNext;
}

State Integrator::simTimeStep(const State &x, const Input &u, const double ts)
{
  // integrate time step
  State x_next = x;
  const int integration_steps = (int)(ts / fine_time_step_);
  if (ts / fine_time_step_ != integration_steps) {
    std::cout << "Warning" << std::endl;
  }
  for (int i = 0; i < integration_steps; i++) x_next = RK4(x_next, u, fine_time_step_);

  // std::cout << x_next.X << " ";
  // std::cout << x_next.Y << " ";
  // std::cout << x_next.phi << " ";
  // std::cout << x_next.vx << " ";
  // std::cout << x_next.vy << " ";
  // std::cout << x_next.r << " ";
  // std::cout << x_next.s << " ";
  // std::cout << x_next.D << " ";
  // std::cout << x_next.delta << " ";
  // std::cout << x_next.B << " ";
  // std::cout << x_next.vs << " ";
  // std::cout << std::endl;

  return x_next;
}
}  // namespace mpcc
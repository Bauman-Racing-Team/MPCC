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

#ifndef MPCC_SOLVER_INTERFACE_H
#define MPCC_SOLVER_INTERFACE_H

#include "config.hpp"
#include "types.hpp"
#include "Params/params.hpp"

#include <array>
namespace mpcc
{
struct OptVariables;
struct solverReturn;

class SolverInterface
{
public:
  virtual solverReturn solveMPC(
    std::array<OptVariables, N + 1> &initialGuess, AcadosParameters parameter_,
    const Bounds &bounds, const Cost &cost) = 0;
  virtual ~SolverInterface() { std::cout << "Deleting Solver Interface" << std::endl; }
};
}  // namespace mpcc

#endif  // MPCC_SOLVER_INTERFACE_H

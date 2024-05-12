/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef MPCC_ACADOS_SIM_INTERFACE_H
#define MPCC_ACADOS_SIM_INTERFACE_H

// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados_sim_solver_acados_mpcc.h"

#include "config.h"
#include "types.h"
#include "Cost/cost.h"
#include "Constraints/bounds.h"
#include "solver_interface.h"

#define NZ ACADOS_MPCC_NZ

namespace mpcc
{
struct OptVariables;

class AcadosSimInterface
{
public:
  State Simulate(const State &state_, const Input &input_, double Tsim);
  ~AcadosSimInterface() { std::cout << "Deleting Acados Sim Interface" << std::endl; }

private:
  acados_mpcc_sim_solver_capsule *capsule;
  int status;

  sim_config *acados_sim_config;
  sim_in *acados_sim_in;
  sim_out *acados_sim_out;
  void *acados_sim_dims;

  // initialization for state values
  double x_current[NX];
  double u0[NU];

  // set parameters
  double p[NP];

  // prepare evaluation
  int n_sim_steps;

  void initSim();

  void setSimInit(const State &state_, const Input &input_, double Tsim);
  void setSimParam(Parameter parameter_);
  State AcadosSim();
  void printSim();
  void freeSim();
};
}  // namespace mpcc
#endif  // MPCC_ACADOS_SIM_INTERFACE_H
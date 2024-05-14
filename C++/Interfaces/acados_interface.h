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

#ifndef MPCC_ACADOS_INTERFACE_H
#define MPCC_ACADOS_INTERFACE_H

// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_acados_mpcc.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "config.h"
#include "types.h"
#include "Cost/cost.h"
#include "Constraints/bounds.h"
#include "solver_interface.h"

#define NZ ACADOS_MPCC_NZ
#define NBX ACADOS_MPCC_NBX
#define NBX0 ACADOS_MPCC_NBX0
#define NBU ACADOS_MPCC_NBU
#define NSBX ACADOS_MPCC_NSBX
#define NSBU ACADOS_MPCC_NSBU
#define NSH ACADOS_MPCC_NSH
#define NSG ACADOS_MPCC_NSG
#define NSPHI ACADOS_MPCC_NSPHI
#define NSHN ACADOS_MPCC_NSHN
#define NSGN ACADOS_MPCC_NSGN
#define NSPHIN ACADOS_MPCC_NSPHIN
#define NSBXN ACADOS_MPCC_NSBXN
#define NSN ACADOS_MPCC_NSN
#define NG ACADOS_MPCC_NG
#define NBXN ACADOS_MPCC_NBXN
#define NGN ACADOS_MPCC_NGN
#define NY0 ACADOS_MPCC_NY0
#define NY ACADOS_MPCC_NY
#define NYN ACADOS_MPCC_NYN
#define NH ACADOS_MPCC_NH
#define NPHI ACADOS_MPCC_NPHI
#define NHN ACADOS_MPCC_NHN
#define NH0 ACADOS_MPCC_NH0
#define NPHIN ACADOS_MPCC_NPHIN
#define NR ACADOS_MPCC_NR

namespace mpcc
{
struct OptVariables;
struct solverReturn;

class AcadosInterface : public SolverInterface
{
public:
  solverReturn solveMPC(
    std::array<OptVariables, N + 1> &initial_guess_, std::array<Parameter, N + 1> parameter_,
    const double *bounds);

  ~AcadosInterface() { std::cout << "Deleting Acados Interface" << std::endl; }

private:
  acados_mpcc_solver_capsule *acados_ocp_capsule;
  double *new_time_steps = NULL;
  int status;

  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;
  ocp_nlp_solver *nlp_solver;
  void *nlp_opts;

  // initial condition
  int idxbx0[NBX0];

  double lbx0[NBX0];
  double ubx0[NBX0];

  // initialization for state values
  double x_init[NX];
  double u0[NU];

  // set parameters
  double p[NP];

  // prepare evaluation
  int NTIMINGS = 1;
  double min_time = 1e12;
  double kkt_norm_inf;
  double elapsed_time;
  int sqp_iter;

  double xtraj[NX * (N + 1)];
  double utraj[NU * N];

  // solve ocp in loop
  int rti_phase = 0;

  void initMPC();

  void setInit(const double *bounds, std::array<OptVariables, N + 1> &initial_guess_);
  void setParam(std::array<Parameter, N + 1> parameter_);
  solverReturn Solve();
  void printSol();
  void getSol();
  void freeSolver();
};
}  // namespace mpcc
#endif  // MPCC_ACADOS_INTERFACE_H
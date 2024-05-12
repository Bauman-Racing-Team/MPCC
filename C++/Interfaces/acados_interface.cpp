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

#include "acados_interface.h"

namespace mpcc
{
void AcadosInterface::initMPC()
{
  // Create acados_ocp_capsule
  acados_ocp_capsule = acados_mpcc_acados_create_capsule();
  if (acados_ocp_capsule == nullptr) {
    // Handle error
    printf("Failed to create acados_ocp_capsule. Exiting.\n");
    exit(1);
  }

  // Allocate new_time_steps array and fill it accordingly
  status = acados_mpcc_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
  if (status) {
    // Handle error
    printf("acados_mpcc_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  // Get pointers to acados objects
  nlp_config = acados_mpcc_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = acados_mpcc_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = acados_mpcc_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = acados_mpcc_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = acados_mpcc_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = acados_mpcc_acados_get_nlp_opts(acados_ocp_capsule);
}

// void AcadosInterface::printSol();
// void AcadosInterface::getSol();
// void AcadosInterface::freeSolver();

void AcadosInterface::setInit(const double *bounds, std::array<OptVariables, N + 1> &initial_guess_)
{
  // initial condition
  idxbx0[0] = 0;
  idxbx0[1] = 1;
  idxbx0[2] = 2;
  idxbx0[3] = 3;
  idxbx0[4] = 4;
  idxbx0[5] = 5;
  idxbx0[6] = 6;
  idxbx0[7] = 7;
  idxbx0[8] = 8;
  idxbx0[9] = 9;
  idxbx0[10] = 10;

  for (int i = 0; i < 11; ++i) {
    lbx0[i] = bounds[i];
  }

  for (int i = 0; i < 11; ++i) {
    lbx0[i] = bounds[i + 11];
  }

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

  for (int i = 0; i < N; i++) {
    Eigen::Map<Eigen::Matrix<double, NX, 1>>(x_init, NX) = stateToVector(initial_guess_[i].xk);
    Eigen::Map<Eigen::Matrix<double, NU, 1>>(u0, NU) = inputToVector(initial_guess_[i].uk);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
}

void AcadosInterface::setParam(std::array<Parameter, N + 1> parameter_)
{
  // set parameters
  for (int i = 0; i < N; ++i) {
    p[0] = parameter_[i].xTrack;
    p[1] = parameter_[i].yTrack;
    p[2] = parameter_[i].phiTrack;
    p[3] = parameter_[i].s0;
    p[4] = parameter_[i].qC;
    p[5] = parameter_[i].qL;
    p[6] = parameter_[i].qVs;
    p[7] = parameter_[i].rdThrottle;
    p[8] = parameter_[i].rdSteeringAngle;
    p[9] = parameter_[i].rdBrakes;
    p[10] = parameter_[i].rdVs;
    acados_mpcc_acados_update_params(acados_ocp_capsule, i, p, NP);
  }
}

solverReturn AcadosInterface::solveMPC(
  std::array<OptVariables, N + 1> &initial_guess_, std::array<Parameter, N + 1> parameter_,
  const double *bounds)
{
  initMPC();
  setInit(bounds, initial_guess_);
  setParam(parameter_);
  return Solve(bounds, initial_guess_);
};

solverReturn AcadosInterface::Solve(
  const double *bounds, std::array<OptVariables, N + 1> &initial_guess_)
{
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
  solverReturn mpcSol;

  for (int ii = 0; ii < NTIMINGS; ii++) {
    // initialize solution
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
    status = acados_mpcc_acados_solve(acados_ocp_capsule);
    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);
    getSol();

    std::array<OptVariables, N + 1> optimal_solution;
    for (int i = 0; i <= N; i++) {
      optimal_solution[i].xk = arrayToState(&xtraj[i * NX]);
    }

    for (int i = 0; i < N; i++) {
      optimal_solution[i].uk = arrayToInput(&utraj[i * NU]);
    }
    optimal_solution[N].uk.setZero();

    mpcSol.mpcHorizon = optimal_solution;
    mpcSol.status = status;
  };
  return mpcSol;
}

void AcadosInterface::getSol()
{
  /* print solution and statistics */
  for (int ii = 0; ii <= nlp_dims->N; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii * NX]);
  for (int ii = 0; ii < nlp_dims->N; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii * NU]);

  // get solution
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

  acados_mpcc_acados_print_stats(acados_ocp_capsule);
}

void AcadosInterface::printSol()
{
  getSol();
  printf("\n--- xtraj ---\n");
  d_print_exp_tran_mat(NX, N + 1, xtraj, NX);
  printf("\n--- utraj ---\n");
  d_print_exp_tran_mat(NU, N, utraj, NU);
  // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

  printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

  if (status == ACADOS_SUCCESS) {
    printf("acados_mpcc_acados_solve(): SUCCESS!\n");
  } else {
    printf("acados_mpcc_acados_solve() failed with status %d.\n", status);
  };

  printf("\nSolver info:\n");
  printf(
    " SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, NTIMINGS,
    min_time * 1000, kkt_norm_inf);
}

void AcadosInterface::freeSolver()
{
  // free solver
  status = acados_mpcc_acados_free(acados_ocp_capsule);
  if (status) {
    printf("acados_mpcc_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = acados_mpcc_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    printf("acados_mpcc_acados_free_capsule() returned status %d. \n", status);
  }
}
}  // namespace mpcc

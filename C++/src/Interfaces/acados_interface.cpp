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

#include "acados_interface.hpp"

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

void AcadosInterface::setInit(const Bounds &bounds, std::array<OptVariables, N + 1> &initialGuess)
{
  // initial state x0
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

  Eigen::Map<Eigen::Matrix<double, NBX0, 1>>(lbx0, NBX0) = initialGuess[0].xk;
  Eigen::Map<Eigen::Matrix<double, NBX0, 1>>(ubx0, NBX0) = initialGuess[0].xk;

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

  // x
  int *idxbx = new int[NBX];

  idxbx[0] = 0;
  idxbx[1] = 1;
  idxbx[2] = 2;
  idxbx[3] = 3;
  idxbx[4] = 4;
  idxbx[5] = 5;
  idxbx[6] = 6;
  idxbx[7] = 7;
  idxbx[8] = 8;
  idxbx[9] = 9;
  idxbx[10] = 10;

  double *lubx = new double[2 * NBX];
  double *lbx = lubx;
  double *ubx = lubx + NBX;

  Eigen::Map<Eigen::Matrix<double, NX, 1>>(lbx, NX) = bounds.stateLowerBounds;
  Eigen::Map<Eigen::Matrix<double, NX, 1>>(ubx, NX) = bounds.stateUpperBounds;

  double *luh = new double[2 * NH];
  double *lh = luh;
  double *uh = luh + NH;

  Eigen::Map<Eigen::Matrix<double, NS, 1>>(lh, NS) = bounds.constraintsLowerBounds;
  Eigen::Map<Eigen::Matrix<double, NS, 1>>(uh, NS) = bounds.constraintsUpperBounds;

  for (int i = 1; i < N; i++) {
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
  }
  delete[] lubx;
  delete[] luh;
  delete[] idxbx;

  // u
  int *idxbu = new int[NBU];

  idxbu[0] = 0;
  idxbu[1] = 1;
  idxbu[2] = 2;
  idxbu[3] = 3;

  double *lubu = new double[2 * NBU];
  double *lbu = lubu;
  double *ubu = lubu + NBU;

  Eigen::Map<Eigen::Matrix<double, NU, 1>>(lbu, NU) = bounds.inputLowerBounds;
  Eigen::Map<Eigen::Matrix<double, NU, 1>>(ubu, NU) = bounds.inputUpperBounds;

  for (int i = 0; i < N; i++) {
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    Eigen::Map<Eigen::Matrix<double, NX, 1>>(x_init, NX) = initialGuess[i].xk;
    Eigen::Map<Eigen::Matrix<double, NU, 1>>(u0, NU) = initialGuess[i].uk;
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
  }
  Eigen::Map<Eigen::Matrix<double, NX, 1>>(x_init, NX) = initialGuess[N].xk;
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
  delete[] lubu;
  delete[] idxbu;
}

void AcadosInterface::setParam(AcadosParameters parameter_)
{
  // set parameters
  for (int i = 0; i <= N; ++i) {
    p[0] = parameter_(xTrackP,i);
    p[1] = parameter_(yTrackP,i);
    p[2] = parameter_(yawTrackP,i);
    p[3] = parameter_(s0P,i);
    p[4] = parameter_(qCP,i);
    p[5] = parameter_(qLP,i);
    p[6] = parameter_(qVsP,i);
    p[7] = parameter_(rdThrottleP,i);
    p[8] = parameter_(rdSteeringAngleP,i);
    p[9] = parameter_(rdBrakesP,i);
    p[10] = parameter_(rdVsP,i);
    acados_mpcc_acados_update_params(acados_ocp_capsule, i, p, NP);
  }

  // slacks
  double *zlumem = new double[4 * NS];
  double *Zl = zlumem + NS * 0;
  double *Zu = zlumem + NS * 1;
  double *zl = zlumem + NS * 2;
  double *zu = zlumem + NS * 3;

  for (int i = 1; i < N; i++) {
    Zl[0] = parameter_(scQuadAlphaFrontP,i);
    Zl[1] = parameter_(scQuadAlphaRearP,i);
    Zl[2] = parameter_(scQuadROutP,i);
    Zl[3] = parameter_(scQuadEllipseFrontP,i);
    Zl[4] = parameter_(scQuadEllipseRearP,i);
    Zl[5] = parameter_(scQuadLonControlP,i);
    Zu[0] = parameter_(scQuadAlphaFrontP,i);
    Zu[1] = parameter_(scQuadAlphaRearP,i);
    Zu[2] = parameter_(scQuadROutP,i);
    Zu[3] = parameter_(scQuadEllipseFrontP,i);
    Zu[4] = parameter_(scQuadEllipseRearP,i);
    Zu[5] = parameter_(scQuadLonControlP,i);
    zl[0] = parameter_(scLinAlphaFrontP,i);
    zl[1] = parameter_(scLinAlphaRearP,i);
    zl[2] = parameter_(scLinROutP,i);
    zl[3] = parameter_(scLinEllipseFrontP,i);
    zl[4] = parameter_(scLinEllipseRearP,i);
    zl[5] = parameter_(scLinLonControlP,i);
    zu[0] = parameter_(scLinAlphaFrontP,i);
    zu[1] = parameter_(scLinAlphaRearP,i);
    zu[2] = parameter_(scLinROutP,i);
    zu[3] = parameter_(scLinEllipseFrontP,i);
    zu[4] = parameter_(scLinEllipseRearP,i);
    zu[5] = parameter_(scLinLonControlP,i);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
  }
  free(zlumem);
}

solverReturn AcadosInterface::solveMPC(
  std::array<OptVariables, N + 1> &initialGuess, AcadosParameters parameter_,
  const Bounds &bounds)
{
  initMPC();
  setInit(bounds, initialGuess);
  setParam(parameter_);
  return Solve();
};

solverReturn AcadosInterface::Solve()
{
  // solve ocp in loop
  int rti_phase = 0;
  solverReturn mpcSol;

  // initialize solution
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
  status = acados_mpcc_acados_solve(acados_ocp_capsule);
  ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
  min_time = MIN(elapsed_time, min_time);
  getSol();

  std::array<OptVariables, N + 1> optimalSolution;
  for (int i = 0; i <= N; i++) {
    optimalSolution[i].xk = arrayToState(&xtraj[i * NX]);
  }

  for (int i = 0; i < N; i++) {
    optimalSolution[i].uk = arrayToInput(&utraj[i * NU]);
  }
  optimalSolution[N].uk.Zero();

  mpcSol.mpcHorizon = optimalSolution;
  mpcSol.status = status;

  // printSol();
  freeSolver();

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
}

void AcadosInterface::printSol()
{
  // acados_mpcc_acados_print_stats(acados_ocp_capsule);
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

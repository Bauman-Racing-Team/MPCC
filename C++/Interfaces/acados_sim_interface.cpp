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

#include "acados_sim_interface.h"

namespace mpcc
{

void AcadosSimInterface::initSim()
{
  status = 0;
  capsule = acados_mpcc_acados_sim_solver_create_capsule();
  status = acados_mpcc_acados_sim_create(capsule);

  if (status) {
    printf("acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  acados_sim_config = acados_mpcc_acados_get_sim_config(capsule);
  acados_sim_in = acados_mpcc_acados_get_sim_in(capsule);
  acados_sim_out = acados_mpcc_acados_get_sim_out(capsule);
  acados_sim_dims = acados_mpcc_acados_get_sim_dims(capsule);
}

void AcadosSimInterface::setSimInit(const State &state_, const Input &input_, double Tsim)
{
  // initial condition
  x_current[0] = state_.X;
  x_current[1] = state_.Y;
  x_current[2] = state_.phi;
  x_current[3] = state_.vx;
  x_current[4] = state_.vy;
  x_current[5] = state_.r;
  x_current[6] = state_.s;
  x_current[7] = state_.D;
  x_current[8] = state_.B;
  x_current[9] = state_.delta;
  x_current[10] = state_.vs;

  // initial value for control input
  u0[0] = input_.dD;
  u0[1] = input_.dDelta;
  u0[2] = input_.dB;
  u0[3] = input_.dVs;
  // set inputs
  sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "x", x_current);
  sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "u", u0);

  double ts = Tsim;
  sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "T", &ts);
}

void AcadosSimInterface::setSimParam(Parameter parameter_)
{
  // set parameters
  p[0] = parameter_.xTrack;
  p[1] = parameter_.yTrack;
  p[2] = parameter_.phiTrack;
  p[3] = parameter_.s0;
  p[4] = parameter_.qC;
  p[5] = parameter_.qL;
  p[6] = parameter_.qVs;
  p[7] = parameter_.rdThrottle;
  p[8] = parameter_.rdSteeringAngle;
  p[9] = parameter_.rdBrakes;
  p[10] = parameter_.rdVs;
  acados_mpcc_acados_sim_update_params(capsule, p, NP);
}

State AcadosSimInterface::Simulate(const State &state_, const Input &input_, double Tsim)
{
  initSim();
  setSimInit(state_, input_, Tsim);
  // setSimParam(parameter_);
  return AcadosSim();
};

State AcadosSimInterface::AcadosSim()
{
  n_sim_steps = 1;
  // solve ocp in loop
  for (int ii = 0; ii < n_sim_steps; ii++) {
    // solve
    status = acados_mpcc_acados_sim_solve(capsule);
    if (status != ACADOS_SUCCESS) {
      printf("acados_solve() failed with status %d.\n", status);
    }
  }
  // get outputs
  sim_out_get(acados_sim_config, acados_sim_dims, acados_sim_out, "x", x_current);
  // printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);
  State x_next;
  x_next.X = x_current[0];
  x_next.Y = x_current[1];
  x_next.phi = x_current[2];
  x_next.vx = x_current[3];
  x_next.vy = x_current[4];
  x_next.r = x_current[5];
  x_next.s = x_current[6];
  x_next.D = x_current[7];
  x_next.B = x_current[8];
  x_next.delta = x_current[9];
  x_next.vs = x_current[10];

  // printSim();
  freeSim();
  return x_next;
}

void AcadosSimInterface::printSim()
{
  for (int ii = 0; ii < n_sim_steps; ii++) {
    // print solution
    printf("\nx_current, %d\n", ii);
    for (int jj = 0; jj < NX; jj++) {
      printf("%e\n", x_current[jj]);
    }
  }
}

void AcadosSimInterface::freeSim()
{
  // free solver
  status = acados_mpcc_acados_sim_free(capsule);
  if (status) {
    printf("acados_mpcc_acados_sim_free() returned status %d. \n", status);
  }

  acados_mpcc_acados_sim_solver_free_capsule(capsule);
}

}  // namespace mpcc

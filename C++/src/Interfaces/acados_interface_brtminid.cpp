#include "acados_interface_brtminid.hpp"

#include "brtminid/acados_solver_acados_mpcc.h"

#include <vector>

namespace mpcc
{

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

AcadosInterfaceBrtMiniD::AcadosInterfaceBrtMiniD(
  const Bounds &bounds, const Config &config, const Cost &costs, double ts)
: AcadosInterface(bounds, config, costs, ts)
{
}

void AcadosInterfaceBrtMiniD::initMPC()
{
  d_acadosOcpCapsule = acados_mpcc_acados_create_capsule();
  if (d_acadosOcpCapsule == nullptr) {
    printf("Failed to create acados_ocp_capsule. Exiting.\n");
    exit(1);
  }

  std::fill(d_newTimeSteps, d_newTimeSteps + d_config.n, d_dt);

  acados_mpcc_solver_capsule *castedAcadosOcpCapsule =
    static_cast<acados_mpcc_solver_capsule *>(d_acadosOcpCapsule);

  d_status = acados_mpcc_acados_create_with_discretization(
    castedAcadosOcpCapsule, d_config.n, d_newTimeSteps);
  if (d_status) {
    printf("acados_mpcc_acados_create() returned status %d. Exiting.\n", d_status);
    exit(1);
  }

  d_nlpConfig = acados_mpcc_acados_get_nlp_config(castedAcadosOcpCapsule);
  d_nlpDims = acados_mpcc_acados_get_nlp_dims(castedAcadosOcpCapsule);
  d_nlpIn = acados_mpcc_acados_get_nlp_in(castedAcadosOcpCapsule);
  d_nlpOut = acados_mpcc_acados_get_nlp_out(castedAcadosOcpCapsule);
  d_nlpSolver = acados_mpcc_acados_get_nlp_solver(castedAcadosOcpCapsule);
  d_nlpOpts = acados_mpcc_acados_get_nlp_opts(castedAcadosOcpCapsule);
}

void AcadosInterfaceBrtMiniD::setInitialValues(std::vector<OptVariables> &initialGuess)
{
  if (initialGuess.size() != static_cast<size_t>(d_config.n + 1)) {
    throw std::runtime_error("Wrong initialGuess size");
  }
  int idxBx0[NBX0];

  idxBx0[0] = 0;
  idxBx0[1] = 1;
  idxBx0[2] = 2;
  idxBx0[3] = 3;
  idxBx0[4] = 4;
  idxBx0[5] = 5;
  idxBx0[6] = 6;
  idxBx0[7] = 7;
  idxBx0[8] = 8;
  idxBx0[9] = 9;
  idxBx0[10] = 10;

  double lBx0[NBX0];
  double uBx0[NBX0];

  Eigen::Map<Eigen::Matrix<double, NBX0, 1>>(lBx0, NBX0) = initialGuess[0].xk;
  Eigen::Map<Eigen::Matrix<double, NBX0, 1>>(uBx0, NBX0) = initialGuess[0].xk;

  ocp_nlp_config *castedNlpConfig = static_cast<ocp_nlp_config *>(d_nlpConfig);
  ocp_nlp_dims *castedNlpDims = static_cast<ocp_nlp_dims *>(d_nlpDims);
  ocp_nlp_in *castedNlpIn = static_cast<ocp_nlp_in *>(d_nlpIn);

  ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, 0, "idxbx", idxBx0);
  ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, 0, "lbx", lBx0);
  ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, 0, "ubx", uBx0);

  int idxBx[NBX];

  idxBx[0] = 0;
  idxBx[1] = 1;
  idxBx[2] = 2;
  idxBx[3] = 3;
  idxBx[4] = 4;
  idxBx[5] = 5;
  idxBx[6] = 6;
  idxBx[7] = 7;
  idxBx[8] = 8;
  idxBx[9] = 9;
  idxBx[10] = 10;

  double luBx[2 * NBX];
  double *lBx = luBx;
  double *uBx = luBx + NBX;

  Eigen::Map<Eigen::Matrix<double, NX, 1>>(lBx, NX) = d_bounds.stateLowerBounds;
  Eigen::Map<Eigen::Matrix<double, NX, 1>>(uBx, NX) = d_bounds.stateUpperBounds;

  double luH[2 * NH];
  double *lH = luH;
  double *uH = luH + NH;

  Eigen::Map<Eigen::Matrix<double, NS, 1>>(lH, NS) = d_bounds.constraintsLowerBounds;
  Eigen::Map<Eigen::Matrix<double, NS, 1>>(uH, NS) = d_bounds.constraintsUpperBounds;

  for (int i = 1; i < d_config.n; i++) {
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "idxbx", idxBx);
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "lbx", lBx);
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "ubx", uBx);
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "lh", lH);
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "uh", uH);
  }

  int idxBu[NBU];

  idxBu[0] = 0;
  idxBu[1] = 1;
  idxBu[2] = 2;
  idxBu[3] = 3;

  double luBu[2 * NBU];
  double *lBu = luBu;
  double *uBu = luBu + NBU;

  Eigen::Map<Eigen::Matrix<double, NU, 1>>(lBu, NU) = d_bounds.inputLowerBounds;
  Eigen::Map<Eigen::Matrix<double, NU, 1>>(uBu, NU) = d_bounds.inputUpperBounds;

  double xInit[NX];
  double u0[NU];

  ocp_nlp_out *castedNlpOut = static_cast<ocp_nlp_out *>(d_nlpOut);

  for (int i = 0; i < d_config.n; i++) {
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "idxbu", idxBu);
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "lbu", lBu);
    ocp_nlp_constraints_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "ubu", uBu);
    Eigen::Map<Eigen::Matrix<double, NX, 1>>(xInit, NX) = initialGuess[i].xk;
    Eigen::Map<Eigen::Matrix<double, NU, 1>>(u0, NU) = initialGuess[i].uk;
    ocp_nlp_out_set(castedNlpConfig, castedNlpDims, castedNlpOut, i, "x", xInit);
    ocp_nlp_out_set(castedNlpConfig, castedNlpDims, castedNlpOut, i, "u", u0);
  }
  Eigen::Map<Eigen::Matrix<double, NX, 1>>(xInit, NX) = initialGuess[d_config.n].xk;
  ocp_nlp_out_set(castedNlpConfig, castedNlpDims, castedNlpOut, d_config.n, "x", xInit);

  double zlumem[4 * NS];
  double *Zl = zlumem + NS * 0;
  double *Zu = zlumem + NS * 1;
  double *zl = zlumem + NS * 2;
  double *zu = zlumem + NS * 3;

  Zl[0] = d_costs.scQuadAlphaFront;
  Zl[1] = d_costs.scQuadAlphaRear;
  Zl[2] = d_costs.scQuadROut;
  Zl[3] = d_costs.scQuadROut;
  Zl[4] = d_costs.scQuadEllipseFront;
  Zl[5] = d_costs.scQuadEllipseRear;
  Zl[6] = d_costs.scQuadLonControl;

  Zu[0] = d_costs.scQuadAlphaFront;
  Zu[1] = d_costs.scQuadAlphaRear;
  Zu[2] = d_costs.scQuadROut;
  Zu[3] = d_costs.scQuadROut;
  Zu[4] = d_costs.scQuadEllipseFront;
  Zu[5] = d_costs.scQuadEllipseRear;
  Zu[6] = d_costs.scQuadLonControl;

  zl[0] = d_costs.scLinAlphaFront;
  zl[1] = d_costs.scLinAlphaRear;
  zl[2] = d_costs.scLinROut;
  zl[3] = d_costs.scLinROut;
  zl[4] = d_costs.scLinEllipseFront;
  zl[5] = d_costs.scLinEllipseRear;
  zl[6] = d_costs.scLinLonControl;

  zu[0] = d_costs.scLinAlphaFront;
  zu[1] = d_costs.scLinAlphaRear;
  zu[2] = d_costs.scLinROut;
  zu[3] = d_costs.scLinROut;
  zu[4] = d_costs.scLinEllipseFront;
  zu[5] = d_costs.scLinEllipseRear;
  zu[6] = d_costs.scLinLonControl;

  for (int i = 1; i < d_config.n; i++) {
    ocp_nlp_cost_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "Zl", Zl);
    ocp_nlp_cost_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "Zu", Zu);
    ocp_nlp_cost_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "zl", zl);
    ocp_nlp_cost_model_set(castedNlpConfig, castedNlpDims, castedNlpIn, i, "zu", zu);
  }
}

void AcadosInterfaceBrtMiniD::setParameters(AcadosParameters parameters)
{
  acados_mpcc_solver_capsule *castedAcadosOcpCapsule =
    static_cast<acados_mpcc_solver_capsule *>(d_acadosOcpCapsule);

  for (int i = 0; i <= d_config.n; ++i) {
    double p[NP];
    p[0] = parameters(xTrackP, i);
    p[1] = parameters(yTrackP, i);
    p[2] = parameters(yawTrackP, i);
    p[3] = parameters(s0P, i);
    p[4] = parameters(vRefP, i);
    p[5] = parameters(qCP, i);
    p[6] = parameters(qLP, i);
    p[7] = parameters(qVsP, i);
    p[8] = parameters(rdThrottleP, i);
    p[9] = parameters(rdSteeringAngleP, i);
    p[10] = parameters(rdBrakesP, i);
    p[11] = parameters(rdVsP, i);
    p[12] = parameters(xOuterBorderP, i);
    p[13] = parameters(yOuterBorderP, i);
    p[14] = parameters(xInnerBorderP, i);
    p[15] = parameters(yInnerBorderP, i);
    acados_mpcc_acados_update_params(castedAcadosOcpCapsule, i, p, NP);
  }
}

solverReturn AcadosInterfaceBrtMiniD::solve()
{
  int rtiPhase = 0;
  solverReturn mpcSol;

  ocp_nlp_config *castedNlpConfig = static_cast<ocp_nlp_config *>(d_nlpConfig);

  ocp_nlp_solver_opts_set(castedNlpConfig, d_nlpOpts, "rti_phase", &rtiPhase);
  d_status =
    acados_mpcc_acados_solve(static_cast<acados_mpcc_solver_capsule *>(d_acadosOcpCapsule));
  double elapsedTime;
  ocp_nlp_get(
    castedNlpConfig, static_cast<ocp_nlp_solver *>(d_nlpSolver), "time_tot", &elapsedTime);
  d_minTime = MIN(elapsedTime, d_minTime);
  generateSolution();

  std::vector<OptVariables> optimalSolution(d_config.n + 1);
  for (int i = 0; i <= d_config.n; i++) {
    optimalSolution[i].xk = arrayToState(&d_xTraj[i * NX]);
  }

  for (int i = 0; i < d_config.n; i++) {
    optimalSolution[i].uk = arrayToInput(&d_uTraj[i * NU]);
  }
  optimalSolution[d_config.n].uk.setZero();

  mpcSol.mpcHorizon = optimalSolution;
  mpcSol.status = d_status;

  freeSolver();

  return mpcSol;
}

void AcadosInterfaceBrtMiniD::generateSolution()
{
  ocp_nlp_config *castedNlpConfig = static_cast<ocp_nlp_config *>(d_nlpConfig);
  ocp_nlp_dims *castedNlpDims = static_cast<ocp_nlp_dims *>(d_nlpDims);
  ocp_nlp_out *castedNlpOut = static_cast<ocp_nlp_out *>(d_nlpOut);

  for (int ii = 0; ii <= castedNlpDims->N; ii++) {
    ocp_nlp_out_get(castedNlpConfig, castedNlpDims, castedNlpOut, ii, "x", &d_xTraj[ii * NX]);
  }

  for (int ii = 0; ii < castedNlpDims->N; ii++) {
    ocp_nlp_out_get(castedNlpConfig, castedNlpDims, castedNlpOut, ii, "u", &d_uTraj[ii * NU]);
  }

  ocp_nlp_out_get(castedNlpConfig, castedNlpDims, castedNlpOut, 0, "kkt_norm_inf", &d_kktNormInf);
  ocp_nlp_get(castedNlpConfig, static_cast<ocp_nlp_solver *>(d_nlpSolver), "sqp_iter", &d_sqpIter);
}

void AcadosInterfaceBrtMiniD::freeSolver()
{
  acados_mpcc_solver_capsule *castedAcadosOcpCapsule =
    static_cast<acados_mpcc_solver_capsule *>(d_acadosOcpCapsule);

  d_status = acados_mpcc_acados_free(castedAcadosOcpCapsule);
  if (d_status) {
    printf("acados_mpcc_acados_free() returned status %d.\n", d_status);
  }

  d_status = acados_mpcc_acados_free_capsule(castedAcadosOcpCapsule);
  if (d_status) {
    printf("acados_mpcc_acados_free_capsule() returned status %d.\n", d_status);
  }
}
}  // namespace mpcc

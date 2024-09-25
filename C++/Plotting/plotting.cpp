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

#include "plotting.h"
namespace mpcc
{

Plotting::Plotting(double Ts, PathToJson path) : param_(Param(path.param_path)) {}
void Plotting::plotRun(const std::list<MPCReturn> &log, const TrackPos &track_xy) const
{
  std::vector<double> plot_xc(track_xy.X.data(), track_xy.X.data() + track_xy.X.size());
  std::vector<double> plot_yc(track_xy.Y.data(), track_xy.Y.data() + track_xy.Y.size());

  std::vector<double> plot_xi(
    track_xy.X_inner.data(), track_xy.X_inner.data() + track_xy.X_inner.size());
  std::vector<double> plot_yi(
    track_xy.Y_inner.data(), track_xy.Y_inner.data() + track_xy.Y_inner.size());
  std::vector<double> plot_xo(
    track_xy.X_outer.data(), track_xy.X_outer.data() + track_xy.X_outer.size());
  std::vector<double> plot_yo(
    track_xy.Y_outer.data(), track_xy.Y_outer.data() + track_xy.Y_outer.size());

  std::vector<double> plot_x;
  std::vector<double> plot_y;
  std::vector<double> plot_phi;
  std::vector<double> plot_vx;
  std::vector<double> plot_vy;
  std::vector<double> plot_r;
  std::vector<double> plot_s;
  std::vector<double> plot_throttle;
  std::vector<double> plot_brakes;
  std::vector<double> plot_steering;
  std::vector<double> plot_vs;

  std::vector<double> plot_dThrottle;
  std::vector<double> plot_dBrakes;
  std::vector<double> plot_dsteering;
  std::vector<double> plot_dvs;

  std::vector<double> plot_alpha_f;
  std::vector<double> plot_tire_rear;
  std::vector<double> plot_tire_front;

  for (MPCReturn log_i : log) {
    plot_x.push_back(log_i.mpc_horizon[0].xk.X);
    plot_y.push_back(log_i.mpc_horizon[0].xk.Y);
    plot_phi.push_back(log_i.mpc_horizon[0].xk.phi);
    plot_vx.push_back(log_i.mpc_horizon[0].xk.vx);
    plot_vy.push_back(log_i.mpc_horizon[0].xk.vy);
    plot_r.push_back(log_i.mpc_horizon[0].xk.r);
    plot_s.push_back(log_i.mpc_horizon[0].xk.s);
    plot_throttle.push_back(log_i.mpc_horizon[0].xk.throttle);
    plot_steering.push_back(log_i.mpc_horizon[0].xk.steeringAngle);
    plot_brakes.push_back(log_i.mpc_horizon[0].xk.brakes);
    plot_vs.push_back(log_i.mpc_horizon[0].xk.vs);

    plot_dThrottle.push_back(log_i.mpc_horizon[0].uk.dThrottle);
    plot_dsteering.push_back(log_i.mpc_horizon[0].uk.dSteeringAngle);
    plot_dBrakes.push_back(log_i.mpc_horizon[0].uk.dBrakes);
    plot_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

    const StateVector x_vec = stateToVector(log_i.mpc_horizon[2].xk);
    const std::vector<double> x_std_vec(x_vec.data(), x_vec.data() + x_vec.size());
    double alpha_f = 0.0;  
    plot_alpha_f.push_back(alpha_f);
  }

  plt::figure();
  plt::plot(plot_xc, plot_yc, "r--");
  plt::plot(plot_xi, plot_yi, "k-");
  plt::plot(plot_xo, plot_yo, "k-");
  plt::plot(plot_x, plot_y, "b-");
  plt::axis("equal");
  plt::xlabel("X [m]");
  plt::ylabel("Y [m]");

  plt::figure();
  plt::plot(plot_x);
  plt::ylabel("X [m]");

  plt::figure();
  plt::plot(plot_y);
  plt::ylabel("Y [m]");

  plt::figure();
  plt::plot(plot_phi);
  plt::ylabel("phi [rad]");

  plt::figure();
  plt::plot(plot_vx);
  plt::ylabel("v_x [m/s]");

  plt::figure();
  plt::plot(plot_vy);
  plt::ylabel("v_y [m/s]");

  plt::figure();
  plt::plot(plot_r);
  plt::ylabel("r [rad/s]");

  plt::figure();
  plt::plot(plot_throttle);
  plt::plot(plot_brakes);
  plt::ylabel("throttle/brakes [-]");

  plt::figure();
  plt::plot(plot_steering);
  plt::ylabel("steering [rad]");

  plt::figure();
  plt::plot(plot_vs);
  plt::ylabel("v_s [m/s]");

  plt::figure();
  plt::plot(plot_dThrottle);
  plt::ylabel("dot{Throttle} [-]");

  plt::figure();
  plt::plot(plot_dBrakes);
  plt::ylabel("dot{Brakes} [-]");

  plt::figure();
  plt::plot(plot_dsteering);
  plt::ylabel("dot{steering} [rad/s]");

  plt::figure();
  plt::plot(plot_dvs);
  plt::ylabel("dot{v_s} [m/s^2]");

  plt::figure();
  plt::plot(plot_s);
  plt::ylabel("s [m]");

  plt::show();
}
void Plotting::plotSim(const std::list<MPCReturn> &log, const TrackPos &track_xy) const
{
  std::vector<double> plot_xc(track_xy.X.data(), track_xy.X.data() + track_xy.X.size());
  std::vector<double> plot_yc(track_xy.Y.data(), track_xy.Y.data() + track_xy.Y.size());

  std::vector<double> plot_xi(
    track_xy.X_inner.data(), track_xy.X_inner.data() + track_xy.X_inner.size());
  std::vector<double> plot_yi(
    track_xy.Y_inner.data(), track_xy.Y_inner.data() + track_xy.Y_inner.size());
  std::vector<double> plot_xo(
    track_xy.X_outer.data(), track_xy.X_outer.data() + track_xy.X_outer.size());
  std::vector<double> plot_yo(
    track_xy.Y_outer.data(), track_xy.Y_outer.data() + track_xy.Y_outer.size());

  std::vector<double> plot_x;
  std::vector<double> plot_y;

  plt::figure();
  for (MPCReturn log_i : log) {
    plot_x.resize(0);
    plot_y.resize(0);
    for (int j = 0; j < log_i.mpc_horizon.size(); j++) {
      plot_x.push_back(log_i.mpc_horizon[j].xk.X);
      plot_y.push_back(log_i.mpc_horizon[j].xk.Y);
    }
    double max_x = *std::max_element(plot_x.begin(), plot_x.end());
    double min_x = *std::min_element(plot_x.begin(), plot_x.end());
    double max_y = *std::max_element(plot_y.begin(), plot_y.end());
    double min_y = *std::min_element(plot_y.begin(), plot_y.end());

    plt::clf();
    plt::plot(plot_xc, plot_yc, "r--");
    plt::plot(plot_xi, plot_yi, "k-");
    plt::plot(plot_xo, plot_yo, "k-");
    plotBox(log_i.mpc_horizon[0].xk);
    plt::plot(plot_x, plot_y, "b-");
    plt::axis("equal");
    plt::xlim(min_x - 5, max_x + 20);
    plt::ylim(min_y - 20, max_y + 20);
    plt::pause(0.01);
  }
}

void Plotting::plotBox(const State &x0) const
{
  std::vector<double> corner_x;
  std::vector<double> corner_y;
  double body_xl = std::cos(x0.phi) * param_.car_l / 2;
  double body_xw = std::sin(x0.phi) * param_.car_w / 2;
  double body_yl = std::sin(x0.phi) * param_.car_l / 2;
  double body_yw = -std::cos(x0.phi) * param_.car_w / 2;

  corner_x.push_back(x0.X + body_xl + body_xw);
  corner_x.push_back(x0.X + body_xl - body_xw);
  corner_x.push_back(x0.X - body_xl - body_xw);
  corner_x.push_back(x0.X - body_xl + body_xw);
  corner_x.push_back(x0.X + body_xl + body_xw);

  corner_y.push_back(x0.Y + body_yl + body_yw);
  corner_y.push_back(x0.Y + body_yl - body_yw);
  corner_y.push_back(x0.Y - body_yl - body_yw);
  corner_y.push_back(x0.Y - body_yl + body_yw);
  corner_y.push_back(x0.Y + body_yl + body_yw);

  plt::plot(corner_x, corner_y, "k-");
}
}  // namespace mpcc
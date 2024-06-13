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

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main()
{
  using namespace mpcc;
  std::string goToPath = "../";
  std::ifstream iConfig(goToPath + "Params/config.json");
  json jsonConfig;
  iConfig >> jsonConfig;

  PathToJson jsonPaths{
    goToPath + std::string(jsonConfig["model_path"]),
    goToPath + std::string(jsonConfig["cost_path"]),
    goToPath + std::string(jsonConfig["bounds_path"]),
    goToPath + std::string(jsonConfig["track_path"])};

  Integrator integrator;
  Plotting plotter = Plotting(jsonConfig["Ts"], jsonPaths);

  Track track = Track(jsonPaths.track_path);
  TrackPos track_xy = track.getTrack();

  std::list<MPCReturn> log;
  MPC mpc(
    jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], jsonConfig["Ts"],
    jsonPaths);
  mpc.setTrack(track_xy.X, track_xy.Y);
  const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0), track_xy.X(1) - track_xy.X(0));
  State x0 = {track_xy.X(0),   track_xy.Y(0), phi_0, jsonConfig["v0"], 0, 0, 0, 0.0, 0, 0,
              jsonConfig["v0"]};
  for (int i = 0; i < jsonConfig["n_sim"]; i++) {
    MPCReturn mpc_sol = mpc.runMPC(x0);
    // Use the MPC prediction as sim step
    x0 = mpc_sol.mpc_horizon[1].xk;

    // Use ODE integrator
    // x0 = integrator.RK4(x0, mpc_sol.u0, jsonConfig["Ts"]);

    log.push_back(mpc_sol);
    std::cout << "MPC iter =  " << i + 1 << std::endl;
    // std::cout << "Solver status=" << mpc_sol.solverStatus << std::endl;
    if (mpc_sol.solverStatus == 4) {
      std::cout << "Solver error 4: QP solver failed." << std::endl;
      break;
    }
  }

  // Plot data
  plotter.plotRun(log, track_xy);
  plotter.plotSim(log, track_xy);

  double mean_time = 0.0;
  double max_time = 0.0;
  for (MPCReturn log_i : log) {
    mean_time += log_i.time_total;
    if (log_i.time_total > max_time) max_time = log_i.time_total;
  }
  std::cout << "mean nmpc time " << mean_time / double(jsonConfig["n_sim"]) << std::endl;
  std::cout << "max nmpc time " << max_time << std::endl;

  return 0;
}
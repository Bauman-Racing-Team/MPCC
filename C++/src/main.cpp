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

#include "MPC/mpc.hpp"
#include "Track/track.hpp"
#include "Params/params.hpp"
#include "Plotting/plotting.hpp"
#include "Simulator/simulator.hpp"

#include <nlohmann/json.hpp>

#include <vector>

using json = nlohmann::json;

int main()
{
  using namespace mpcc;
  std::string goToPath = "../data/";
  std::ifstream iConfig(goToPath + "params/config.json");
  json jsonConfig;
  iConfig >> jsonConfig;

  PathToJson jsonPaths{
    goToPath + std::string(jsonConfig["model_path"]),
    goToPath + std::string(jsonConfig["costs_path"]),
    goToPath + std::string(jsonConfig["bounds_path"]),
    goToPath + std::string(jsonConfig["track_path"]),
    goToPath + std::string(jsonConfig["car_path"]),
    goToPath + std::string(jsonConfig["tire_path"])};

  Plotting plotter = Plotting(jsonConfig["Ts"], jsonPaths);

  Track track = Track(jsonPaths.trackPath);
  TrackPos trackXY = track.getTrack();

  std::vector<MPCReturn> log;
  
  MPC mpc(
    jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], jsonConfig["Ts"],
    jsonPaths);
  
  mpc.setTrack(trackXY.X, trackXY.Y);
  
  double yaw0 = std::atan2(trackXY.Y(1) - trackXY.Y(0), trackXY.X(1) - trackXY.X(0));
  
  State13 x0 = {trackXY.X(0),   trackXY.Y(0), yaw0, jsonConfig["v0"], 0., 0., 0., 0., 0., 0.,
              jsonConfig["v0"], 0., 0.};
  
  Simulator simulator(jsonPaths, mpc.getTrack());
  
  for (int i = 0; i < jsonConfig["n_sim"]; i++) {
    MPCReturn mpcSol = mpc.runMPC(x0.head<NX>());

    std::cout << "MPC compute time: " << mpcSol.time_total;

    // Use ODE integrator
    x0 = simulator.simTimeStep(x0, mpcSol.u0, jsonConfig["Ts"]);

    log.push_back(mpcSol);
    std::cout << "MPC iter =  " << i + 1 << std::endl;
    if(mpcSol.solverStatus != 0){
      std::cout << "Solver status=" << mpcSol.solverStatus << std::endl;
    }
    
    if (mpcSol.solverStatus == 4) {
      std::cout << "Solver error 4: QP solver failed." << std::endl;
      break;
    }
  }

  // Plot data
  plotter.plotRun(log, trackXY);
  plotter.plotSim(log, trackXY);

  double meanTime = 0.0;
  double maxTime = 0.0;

  for (MPCReturn logI : log) {
    meanTime += logI.time_total;
    if (logI.time_total > maxTime) maxTime = logI.time_total;
  }
  std::cout << "mean nmpc time " << meanTime / double(jsonConfig["n_sim"]) << std::endl;
  std::cout << "max nmpc time " << maxTime << std::endl;

  return 0;
}
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

static const std::string AUTONOMOUS_VEHICLE = "brt9d"; // brt8d, brt9d, brtminid
static const std::string TRACK = "FSG";

static constexpr int SIM_ITERATIONS = 3000; // [i] simulation iterations number
static constexpr double Ts = 0.05; // [s] MPCC computation dt

int main()
{
  using namespace mpcc;

  std::string dataPath = "../../data";
  std::string trackPath = dataPath + "/tracks/" + TRACK + ".json";
  std::string carPath = dataPath + "/cars/" + AUTONOMOUS_VEHICLE + "/car.json";
  std::string paramsPath = dataPath + "/params/" + AUTONOMOUS_VEHICLE;
  std::string tirePath = dataPath + "/cars/" + AUTONOMOUS_VEHICLE + "/tire.json";

  std::string boundsPath = paramsPath + "/bounds.json";
  std::string configPath = paramsPath + "/mpcc.json";
  std::string costPath = paramsPath + "/cost.json";

  Car car(carPath);
  Tire tire(tirePath);

  Plotting plotter = Plotting(Ts, carPath);

  Track track = Track(trackPath);
  TrackPos trackXY = track.getTrack();

  std::vector<MPCReturn> log;
  
  Bounds bounds(boundsPath);
  Config config(configPath);
  Cost cost(costPath);

  MPC mpc(AUTONOMOUS_VEHICLE, bounds, config, cost, car, tire, Ts);
  mpc.setTrack(trackXY.X, trackXY.Y, trackXY.X_outer, trackXY.Y_outer, trackXY.X_inner, trackXY.Y_inner);

  double yaw0 = std::atan2(trackXY.Y(1) - trackXY.Y(0), trackXY.X(1) - trackXY.X(0));

  State13 x0 = {trackXY.X(0),   trackXY.Y(0), yaw0, 0., 0., 0., 0., 0., 0., 0.,
              0., 0., 0.};
  
  Simulator simulator(car, tire, mpc.getTrack());
  
  for (int i = 0; i < SIM_ITERATIONS; i++) {
    MPCReturn mpcSol = mpc.runMPC(x0.head<NX>());

    std::cout << "MPC compute time: " << mpcSol.time_total << " ";

    // Use ODE integrator
    x0 = simulator.simTimeStep(x0, mpcSol.u0, Ts);

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
  std::cout << "mean nmpc time " << meanTime / double(SIM_ITERATIONS) << std::endl;
  std::cout << "max nmpc time " << maxTime << std::endl;

  return 0;
}
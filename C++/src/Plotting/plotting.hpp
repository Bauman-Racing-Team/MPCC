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

#ifndef MPCC_PLOTTING_H
#define MPCC_PLOTTING_H

#include "config.hpp"
#include "types.hpp"
#include "Params/params.hpp"
#include "Track/track.hpp"
#include "MPC/mpc.hpp"
#include "MPC/mpc_track.hpp"

#include <matplotlibcpp.h>

#include <optional>
#include <vector>

namespace plt = matplotlibcpp;

namespace mpcc {
class Plotting {
public:
    void plotRun(const std::vector<MPCReturn> &log, const TrackPos &track_xy, const MpcTrack& mpcTrack) const;
    void plotSim(const std::vector<MPCReturn> &log, const TrackPos &track_xy, const MpcTrack& mpcTrack) const;

    Plotting(double Ts, const Car& car);

private:
    void plotTrack(const TrackPos &track_xy, std::optional<long> fig = std::nullopt) const;
    void plotMpcTrack(const MpcTrack& mpcTrack, std::optional<long> fig = std::nullopt) const;
    void plotBox(const State &x0) const;

    Car d_car;
};
}

#endif //MPCC_PLOTTING_H

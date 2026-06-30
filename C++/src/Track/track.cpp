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

#include "track.hpp"
namespace mpcc{
Track::Track(std::string file) 
{
    /////////////////////////////////////////////////////
    // Loading Model and Constraint Parameters //////////
    /////////////////////////////////////////////////////
    std::ifstream iTrack(file);
    json jsonTrack;
    iTrack >> jsonTrack;
    // Model Parameters
    std::vector<double> x = jsonTrack["X"];
    x.push_back(x[0]);
    X = Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
    std::vector<double> y = jsonTrack["Y"];
    y.push_back(y[0]);
    Y = Eigen::Map<Eigen::VectorXd>(y.data(), y.size());
    
    std::vector<double> x_inner = jsonTrack["X_i"];
    x_inner.push_back(x_inner[0]);
    X_inner = Eigen::Map<Eigen::VectorXd>(x_inner.data(), x_inner.size());
    std::vector<double> y_inner = jsonTrack["Y_i"];
    y_inner.push_back(y_inner[0]);
    Y_inner = Eigen::Map<Eigen::VectorXd>(y_inner.data(), y_inner.size());

    std::vector<double> x_outer = jsonTrack["X_o"];
    x_outer.push_back(x_outer[0]);
    X_outer = Eigen::Map<Eigen::VectorXd>(x_outer.data(), x_outer.size());
    std::vector<double> y_outer = jsonTrack["Y_o"];
    y_outer.push_back(y_outer[0]);
    Y_outer = Eigen::Map<Eigen::VectorXd>(y_outer.data(), y_outer.size());
}

TrackPos Track::getTrack()
{
    return {1*Eigen::Map<Eigen::VectorXd>(X.data(), X.size()), 1*Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size()),
            1*Eigen::Map<Eigen::VectorXd>(X_inner.data(), X_inner.size()), 1*Eigen::Map<Eigen::VectorXd>(Y_inner.data(), Y_inner.size()),
            1*Eigen::Map<Eigen::VectorXd>(X_outer.data(), X_outer.size()), 1*Eigen::Map<Eigen::VectorXd>(Y_outer.data(), Y_outer.size())};
}
}

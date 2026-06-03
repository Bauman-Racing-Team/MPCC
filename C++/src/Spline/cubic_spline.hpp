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

#ifndef MPCC_CUBIC_SPLINE_H
#define MPCC_CUBIC_SPLINE_H

#include "config.hpp"
#include <map>

namespace mpcc{
// spline parameter struct y = a + b dx + c dx^2 + d dx^3
struct SplineParams{
    Eigen::VectorXd a;
    Eigen::VectorXd b;
    Eigen::VectorXd c;
    Eigen::VectorXd d;
};
// input data for spline
struct SplineData{
    Eigen::VectorXd xData;     //x data
    Eigen::VectorXd yData;     //y data
    int nPoints;          //number of points
    bool isRegular;    //regular (1) or irregular (0) spaced points in x direction
    double deltaX;  //spacing of regular space points
    std::map<double,int> xMap;
};

class CubicSpline {
public:
    void genSpline(const Eigen::VectorXd &xIn,const Eigen::VectorXd &yIn,bool isRegular);
    double getPoint(double x) const;
    double getDerivative(double x) const;
    double getSecondDerivative(double x) const;

    CubicSpline();
private:
    bool data_set_;

    void setRegularData(const Eigen::VectorXd &xIn,const Eigen::VectorXd &yIn,double deltaX);
    void setData(const Eigen::VectorXd &xIn,const Eigen::VectorXd &yIn);
    bool compSplineParams();
    int getIndex(double x) const;
    double unwrapInput(double x) const;

    SplineParams splineParams;
    SplineData splineData;
};
}
#endif //MPCC_CUBIC_SPLINE_H
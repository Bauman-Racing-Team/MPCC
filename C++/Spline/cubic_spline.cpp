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

#include "cubic_spline.hpp"

namespace mpcc{
CubicSpline::CubicSpline()
: data_set_(false)
{
}

void CubicSpline::setRegularData(const Eigen::VectorXd &xIn,const Eigen::VectorXd &yIn,const double deltaX) {
    //if x and y have same length, stare given data in spline data struct
    if(xIn.size() == yIn.size())
    {
        splineData.xData = xIn;
        splineData.yData = yIn;
        splineData.nPoints = xIn.size();
        splineData.isRegular = true;
        splineData.deltaX = deltaX;

        data_set_ = true;
    }
    else
    {
        std::cout << "input data does not have the same length" << std::endl;
    }
}

void CubicSpline::setData(const Eigen::VectorXd &xIn,const Eigen::VectorXd &yIn)
{
    //if x and y have same length, stare given data in spline data struct
    if(xIn.size() == yIn.size())
    {
        splineData.xData = xIn;
        splineData.yData = yIn;
        splineData.nPoints = xIn.size();
        splineData.isRegular = false;
        splineData.deltaX = 0;
        for(int i = 0;i<xIn.size();i++){
            splineData.xMap[xIn(i)] = i;
        }

        data_set_ = true;
    }
    else
    {
        std::cout << "input data does not have the same length" << std::endl;
    }
}

bool CubicSpline::compSplineParams()
{
    // compute spline parameters parameters
    // code is a replica of the wiki code
    if(!data_set_)
    {
        return false;
    }
    // spline parameters from parameter struct initialized to zero
    splineParams.a.setZero(splineData.nPoints);
    splineParams.b.setZero(splineData.nPoints-1);
    splineParams.c.setZero(splineData.nPoints);
    splineParams.d.setZero(splineData.nPoints-1);

    // additional variables used to compute a,b,c,d
    Eigen::VectorXd mu, h, alpha, l, z;
    mu.setZero(splineData.nPoints-1);
    h.setZero(splineData.nPoints-1);
    alpha.setZero(splineData.nPoints-1);
    l.setZero(splineData.nPoints);
    z.setZero(splineData.nPoints);

    // a is equal to y data
    splineParams.a = splineData.yData;
    // compute h as diff of x data
    for(int i = 0;i<splineData.nPoints-1;i++)
    {
        h(i) = splineData.xData(i+1) - splineData.xData(i);
    }
    // compute alpha
    for(int i = 1;i<splineData.nPoints-1;i++)
    {
        alpha(i) = 3.0/h(i)*(splineParams.a(i+1) - splineParams.a(i)) - 3.0/h(i-1)*(splineParams.a(i) - splineParams.a(i-1));
    }

    // compute l, mu, and z
    l(0) = 1.0;
    mu(0) = 0.0;
    z(0) = 0.0;
    for(int i = 1;i<splineData.nPoints-1;i++)
    {
        l(i) = 2.0*(splineData.xData(i+1) - splineData.xData(i-1)) - h(i-1)*mu(i-1);
        mu(i) = h(i)/l(i);
        z(i) = (alpha(i) - h(i-1)*z(i-1))/l(i);
    }
    l(splineData.nPoints-1) = 1.0;
    z(splineData.nPoints-1) = 0.0;

    // compute b,c,d data given the previous work
    splineParams.c(splineData.nPoints-1) = 0.0;

    for(int i = splineData.nPoints-2;i>=0;i--)
    {
        splineParams.c(i) = z(i) - mu(i)*splineParams.c(i+1);
        splineParams.b(i) = (splineParams.a(i+1) - splineParams.a(i))/h(i) - (h(i)*(splineParams.c(i+1) + 2.0*splineParams.c(i)))/3.0;
        splineParams.d(i) = (splineParams.c(i+1) - splineParams.c(i))/(3.0*h(i));
    }

    return true;
}

int CubicSpline::getIndex(const double x) const
{
    // given a x value find the closest point in the spline to evalute it
    // special case if x is regularly space
    // assumes wrapped data!

    // if special case of end points
    if(x == splineData.xData(splineData.nPoints-1))
    {
        return splineData.nPoints-1;
    }
    // if regular index can be found by rounding
    if(splineData.isRegular == 1)
    {
        return int(floor(x/splineData.deltaX));
    }
    // if irregular index need to be searched
    else
    {
        auto min_it = splineData.xMap.upper_bound(x);
        if(min_it==splineData.xMap.end())
            return -1;
        else{
            return min_it->second-1;
        }

    }
}
double CubicSpline::unwrapInput(double x) const
{
    double x_max = splineData.xData(splineData.nPoints-1);
    return x - x_max*std::floor(x/x_max);
}

void CubicSpline::genSpline(const Eigen::VectorXd &xIn,const Eigen::VectorXd &yIn,const bool isRegular)
{
    // given x and y data generate spline
    // special case for regular or irregular spaced data points in x
    // if regular the spacing in x is given by deltaX

    // store data in data struct
    if(isRegular)
    {
        double deltaX = xIn(1) - xIn(0);
        setRegularData(xIn,yIn,deltaX);
    }
    else
    {
        setData(xIn,yIn);
    }
    // given data compute spline parameters

    bool succes = compSplineParams();

    // TODO if succes is flase call exeption
}

double CubicSpline::getPoint(double x) const
{
    // evaluation of spline a x
    int index;
    double x_i;
    double dx,dx2,dx3;
    // wrape input to data -> x data needs start at 0 and contain end point!!!
    x = unwrapInput(x);
    // compute index
    index = getIndex(x);
    // access previous points
    x_i = splineData.xData(index);
    // compute diff to point and it's powers
    dx = x-x_i;
    dx2 = dx*dx;
    dx3 = dx*dx2;
    // return spline value y = a + b dx + c dx^2 + d dx^3
    return splineParams.a(index) + splineParams.b(index)*dx + splineParams.c(index)*dx2 + splineParams.d(index)*dx3;
}

double CubicSpline::getDerivative(double x) const
{
    // evaluate first derivative of spline
    // identical to noram spline with y' = b + 2 c dx + 3 d dx^2
    int index;
    double x_i;
    double dx,dx2;

    x = unwrapInput(x);
    index = getIndex(x);
    x_i = splineData.xData(index);

    dx = x-x_i;
    dx2 = dx*dx;
    // y' = b + 2 c dx + 3 d dx^2
    return splineParams.b(index) + 2.0*splineParams.c(index)*dx + 3.0*splineParams.d(index)*dx2;
}

double CubicSpline::getSecondDerivative(double x) const
{
    // evaluate second derivative of spline
    // identical to noram spline with y' = 2 c + 6 d dx
    int index;
    double x_i;
    double dx;

    x = unwrapInput(x);
    index = getIndex(x);
    x_i = splineData.xData(index);

    dx = x-x_i;
    // y' = 2 c + 6 d dx
    return 2.0*splineParams.c(index) + 6.0*splineParams.d(index)*dx;
}
}
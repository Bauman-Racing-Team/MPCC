#pragma once

#include "Spline/arc_length_spline.hpp"
#include "types.hpp"

namespace mpcc
{

class MpcTrack {

public:
	MpcTrack(const Config &config);

	inline ArcLengthSpline getCenterLine() const
	{
		return d_centerLine;
	}

	inline ArcLengthSpline getOuterBorder() const
	{
		return d_outerBorder;
	}

	inline ArcLengthSpline getInnerBorder() const
	{
		return d_innerBorder;
	}

	void generate(const Eigen::VectorXd& x, const Eigen::VectorXd& y, 
				  const Eigen::VectorXd& xOuter, const Eigen::VectorXd& yOuter, 
				  const Eigen::VectorXd& xInner, const Eigen::VectorXd& yInner);

private:
	void calculateBordersInterpolations();
	std::pair<double, double> findRayBorderIntersection(double cx, double cy, double nx, double ny, const Eigen::VectorXd& bx, const Eigen::VectorXd& by);

private:
  	ArcLengthSpline d_centerLine;
  	ArcLengthSpline d_outerBorder;
  	ArcLengthSpline d_innerBorder;
};
} // namespace mpcc
#ifndef LIE_BSPLINE
#define LIE_BSPLINE

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include <vector>
#include <map>

class BSpline
{
public:
    BSpline() = default;
    Sophus::SE3d intervalAtoB(const Sophus::SE3d &a, const Sophus::SE3d &b);
    Sophus::SE3d cumulativeForm1(const Sophus::SE3d &T_1, const Sophus::SE3d &T_2,
								 const Sophus::SE3d &T_3, const Sophus::SE3d &T_4, double u);
    void SE3Eigen2Sophus(const Eigen::Isometry3d e, Sophus::SE3d &s);
    bool addKnot( const Sophus::SE3d &new_cp);
    Sophus::SE3d getCurPose(const double &ts);
    bool setTimeInterval(const double &t);
    std::vector<Sophus::SE3d> &getControlPoints();

private:
    double time_interval_ = 1.;
    std::vector<Sophus::SE3d,std::allocator<Sophus::SE3d>> control_points_;
};


#endif
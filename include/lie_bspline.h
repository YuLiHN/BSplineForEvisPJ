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
    Eigen::Isometry3d cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
									 Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u);
    Sophus::SE3d cumulativeForm1(const Sophus::SE3d &T_1, const Sophus::SE3d &T_2,
								 const Sophus::SE3d &T_3, const Sophus::SE3d &T_4, double u);
    void SE3Eigen2Sophus(const Eigen::Isometry3d e, Sophus::SE3d &s);
    bool addKnot(const double &ts, const Sophus::SE3d &new_cp);
    Sophus::SE3d getCurPose(const double &ts);
    bool setTimeInterval(const double &t);

private:
    Sophus::SE3d t1,t2,t3,t4;
    double _time_interval = 1.;
    std::vector<Sophus::SE3d> _control_points;
};


#endif
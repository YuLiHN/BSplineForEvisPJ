#include "lie_bspline.h"

Sophus::SE3d BSpline::intervalAtoB(const Sophus::SE3d &a, const Sophus::SE3d &b)
{
    return Sophus::SE3d(a.inverse()*b);
}

Sophus::SE3d BSpline::cumulativeForm1(const Sophus::SE3d &T_1, const Sophus::SE3d &T_2,
								      const Sophus::SE3d &T_3, const Sophus::SE3d &T_4, double u)
{
	Sophus::SE3d res;
	res =   Sophus::SE3d::exp(T_1.log())*
			Sophus::SE3d::exp(((5 + 3*u - 3*u*u + u*u*u) / 6)*intervalAtoB(T_1,T_2).log())*
			Sophus::SE3d::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6)*intervalAtoB(T_2,T_3).log())*
		    Sophus::SE3d::exp(((u*u*u)/6)*intervalAtoB(T_3,T_4).log());

	return res;
}

void BSpline::SE3Eigen2Sophus(const Eigen::Isometry3d e, Sophus::SE3d &s) 
{
	Sophus::SE3d t1;
	Eigen::Matrix4d temp;
	temp = e.matrix();
	Eigen::Vector3d t(temp(0,3),temp(1,3),temp(2,3));
	t1 = Sophus::SE3d(e.rotation(),t);
	s = t1;
}

bool BSpline::addKnot( const Sophus::SE3d &new_cp)
{
	control_points_.push_back(new_cp);
	return true;
}
Sophus::SE3d BSpline::getCurPose(const double &ts)
{
	// if(ts<_time_interval || ts>_time_interval * (_control_points.size()-1))
	// 	return false;
	int i = ts/time_interval_;
	double u = (ts - i * time_interval_)/time_interval_;
	// std::cout<<"i"<<i<<std::endl;
	// std::cout<<"u"<<u<<std::endl;
	return cumulativeForm1(control_points_[i-1],control_points_[i],control_points_[i+1],control_points_[i+2], u);
}
bool BSpline::setTimeInterval(const double &t)
{
	time_interval_ = t;
	return true;
}

std::vector<Sophus::SE3d> &BSpline::getControlPoints()
{
	return control_points_;
}




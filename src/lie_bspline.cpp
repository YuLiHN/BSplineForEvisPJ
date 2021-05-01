#include "lie_bspline.h"

Sophus::SE3d BSpline::intervalAtoB(const Sophus::SE3d &a, const Sophus::SE3d &b)
{
    return Sophus::SE3d(a.inverse()*b);
}

Eigen::Isometry3d BSpline::cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
										  Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u) 
{
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	Sophus::SE3d cur;
	SE3Eigen2Sophus(T_1,t1);
	SE3Eigen2Sophus(T_2,t2);
	SE3Eigen2Sophus(T_3,t3);
	SE3Eigen2Sophus(T_4,t4);
	cur =   Sophus::SE3d::exp(t1.log())*
			Sophus::SE3d::exp(((5 + 3*u - 3*u*u + u*u*u) / 6)*intervalAtoB(t1,t2).log())*
			Sophus::SE3d::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6)*intervalAtoB(t2,t3).log())*
		    Sophus::SE3d::exp(((u*u*u)/6)*intervalAtoB(t3,t4).log());
	result = cur.matrix();
	return result;
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

bool BSpline::addKnot(const double &ts, const Sophus::SE3d &new_cp)
{
	_control_points.push_back(new_cp);
	return true;
}
Sophus::SE3d BSpline::getCurPose(const double &ts)
{
	// if(ts<_time_interval || ts>_time_interval * (_control_points.size()-1))
	// 	return false;
	int i = ts/_time_interval;
	double u = (ts - i * _time_interval)/_time_interval;
	std::cout<<"u"<<u<<std::endl;;
	return cumulativeForm1(_control_points[i-1],_control_points[i],_control_points[i+1],_control_points[i+2], u);
}
bool BSpline::setTimeInterval(const double &t)
{
	_time_interval = t;
	return true;
}
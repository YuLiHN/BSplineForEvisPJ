#include "lie_bspline.h"

using std::cout;
using std::endl;
int main(int argc, char *argv[])
{
    BSpline bs;
    //std::cout<< "yes" <<std::endl;
    Eigen::Isometry3d temp_pose1;
	Eigen::Isometry3d temp_pose2;
	Eigen::Isometry3d temp_pose3;
	Eigen::Isometry3d temp_pose4;
	Eigen::Isometry3d temp_pose5;
	Eigen::Isometry3d temp_pose6;
	Eigen::Isometry3d temp_pose7;
	temp_pose1.setIdentity();
	temp_pose1.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)).toRotationMatrix());
	temp_pose1.translate(Eigen::Vector3d(0,0,0.5));
	cout<<"pose1"<<temp_pose1.matrix()<<endl;
	temp_pose2.setIdentity();
	temp_pose2.rotate(Eigen::AngleAxisd(1.5*M_PI/4, Eigen::Vector3d(0,.1,1)).toRotationMatrix());
	temp_pose2.translate(Eigen::Vector3d(0,0,1));
	cout<<"pose2"<<temp_pose2.matrix()<<endl;
	temp_pose3.setIdentity();
	temp_pose3.rotate(Eigen::AngleAxisd(2.2*M_PI/4, Eigen::Vector3d(0,.3,1)).toRotationMatrix());
	temp_pose3.translate(Eigen::Vector3d(0,0,2.3));
	cout<<"pose3"<<temp_pose3.matrix()<<endl;
	temp_pose4.setIdentity();
	temp_pose4.rotate(Eigen::AngleAxisd(3.2*M_PI/4, Eigen::Vector3d(.5,0,1)).toRotationMatrix());
	temp_pose4.translate(Eigen::Vector3d(0,0,3.7));
	cout<<"pose4"<<temp_pose4.matrix()<<endl;
	temp_pose5.setIdentity();
	temp_pose5.rotate(Eigen::AngleAxisd(2.1*M_PI/4, Eigen::Vector3d(.2,0,1)).toRotationMatrix());
	temp_pose5.translate(Eigen::Vector3d(0,1,3.7));
	cout<<"pose5"<<temp_pose5.matrix()<<endl;
	temp_pose6.setIdentity();
	temp_pose6.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,-0.2,1)).toRotationMatrix());
	temp_pose6.translate(Eigen::Vector3d(1,1,3.7));
	cout<<"pose6"<<temp_pose6.matrix()<<endl;
	temp_pose7.setIdentity();
	temp_pose7.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0.4,0,1)).toRotationMatrix());
	temp_pose7.translate(Eigen::Vector3d(1,1.5,3.7));
	cout<<"pose7"<<temp_pose7.matrix()<<endl;


	Eigen::Isometry3d tf1;
	Eigen::Isometry3d tf2;
	Sophus::SE3d tf3;
	Sophus::SE3d temp_p1,temp_p2,temp_p3,temp_p4,temp_p5,temp_p6,temp_p7;
	bs.SE3Eigen2Sophus(temp_pose1,temp_p1);
	bs.SE3Eigen2Sophus(temp_pose2,temp_p2);
	bs.SE3Eigen2Sophus(temp_pose3,temp_p3);
	bs.SE3Eigen2Sophus(temp_pose4,temp_p4);
	bs.SE3Eigen2Sophus(temp_pose5,temp_p5);
	bs.SE3Eigen2Sophus(temp_pose6,temp_p6);
	bs.SE3Eigen2Sophus(temp_pose7,temp_p7);

	tf3 = bs.cumulativeForm1(temp_p1,temp_p2,temp_p3,temp_p4,0.00);
	cout<<tf3.matrix()<<endl;

	BSpline bs2;
	bs.addKnot(temp_p1);
	bs.addKnot(temp_p2);
	bs.addKnot(temp_p3);
	bs.addKnot(temp_p4);
	bs.addKnot(temp_p5);
	bs.addKnot(temp_p6);
	bs.addKnot(temp_p7);
	for(int i=0;i<39;i++)
	{
		double ts = 1. + double(i)/10;
		cout<<ts<<endl;
		cout<<bs.getCurPose(ts).matrix()<<endl;
	}
}
#include "lie_bspline.h"
#include <fstream>
#include <string>

using std::cout;

void loadPoses(BSpline &bs)
{
    std::ifstream input_file;
    input_file.open("../poses.txt");
    // Open file to read data
    if (input_file.is_open())
    {
        cout << "Control poses file opened"<<"\n";

        int count = 0;
        std::string line;
        std::ofstream outgt("gt.txt");

        while( getline(input_file, line) )
        {
            std::istringstream stm(line);
            if(count % 10 == 0){

            long sec, nsec;
            double x, y, z, qx, qy, qz, qw;
            if (stm >> sec >> nsec >> x >> y >> z >> qx >> qy >> qz >> qw)
            {
            const Eigen::Vector3d position(count,0.,0.);
            const Eigen::Quaterniond quat(qw,qx,qy,qz);
            const Sophus::SE3d T(quat, position);
            Sophus::SO3d R(quat);
            Eigen::Matrix3d Re= R.matrix();
            outgt<<Re(0,0)<<" "<<Re(0,1)<<" "<<Re(0,2)<<" "
                <<Re(1,0)<<" "<<Re(1,1)<<" "<<Re(1,2)<<" "
                <<Re(2,0)<<" "<<Re(2,1)<<" "<<Re(2,2)<<"\n";

            if (!bs.addKnot(T)){
                cout<<"add Knot failed!"<<"\n";
            }
            }
            }
            count++;
        }
        input_file.close();
    }
    else
    {
        cout<<"file not found!"<<"\n";
    }
}

int main()
{
    BSpline bs;
    loadPoses(bs);
    std::ofstream out("interpolation.txt");
    //cout<<bs.getControlPoints().size();
    for(int i=0;i<2470;i++)
	{
		double ts = 1. + double(i)/10;
		cout<<ts<<"\n";
        Eigen::Matrix4d temp_pose = bs.getCurPose(ts).matrix();
		cout<<temp_pose<<"\n";
        out<<ts<<" "<<temp_pose(0,0)<<" "<<temp_pose(0,1)<<" "<<temp_pose(0,2)<<" "<<temp_pose(0,3)<<" "
            <<temp_pose(1,0)<<" "<<temp_pose(1,1)<<" "<<temp_pose(1,2)<<" "<<temp_pose(1,3)<<" "
            <<temp_pose(2,0)<<" "<<temp_pose(2,1)<<" "<<temp_pose(2,2)<<" "<<temp_pose(2,3)<<" "
            <<temp_pose(3,0)<<" "<<temp_pose(3,1)<<" "<<temp_pose(3,2)<<" "<<temp_pose(3,3)<<"\n";

	}

}
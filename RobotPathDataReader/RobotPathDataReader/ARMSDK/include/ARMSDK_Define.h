#ifndef _ARMSDK_DEFINE_H_
#define _ARMSDK_DEFINE_H_

#include <vector>
#include <Eigen\Dense>

using namespace std;

typedef Eigen::MatrixXd matd;
typedef Eigen::MatrixXi mati;

typedef Eigen::VectorXd vecd;
typedef Eigen::VectorXi veci;

namespace armsdk
{
	enum
	{
		Enable=0,
		Disable = 1
	};

	enum
	{
		PtoP     = 0,
		Linear   = 1,
		Circular = 2
	};

	typedef struct
	{
		double x, y, z;
		double Roll, Pitch, Yaw;
	}Pose3D;

	typedef struct
	{
		double x, y, z;
	}Position3D;

	typedef struct
	{
		double ta, tc, td, totoaltime;
		double a0[3], a1[3], a2[3];
		double distance;  // if Mehod is Linear, distance is Length. else, distance is angular distance
		double distance1; // for circular motion
		int Method;
	} timeprofile;

	typedef struct
	{
		vecd StartPose, EndPose;
		Pose3D StartPose3D, ViaPose3D, EndPose3D;
		Position3D CenterPosition;
	} MotionPose;

	typedef vector< vector<timeprofile> > MotionProfile;
	typedef vector< MotionPose > MotionPoseList;
}

#endif
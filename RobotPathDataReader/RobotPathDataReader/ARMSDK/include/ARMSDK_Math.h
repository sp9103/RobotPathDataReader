#ifndef __ARMSDK_MATH_H_
#define __ARMSDK_MATH_H_

#include "ARMSDK_Define.h"

#ifndef ML_PI
#define ML_PI  (3.14159265358979323846)
#define ML_2PI (ML_PI*2.0)
#define ML_PI_2 (ML_PI/2.0)
#define ML_PI_4 (ML_PI/4.0)
#define _RADtoDEG  (180.0/ML_PI)
#define _DEGtoRAD  (ML_PI/180.0)
#endif

namespace armsdk
{
	class Algebra
	{
	public:
		static matd GetOrientationMatrix(double Roll, double Pitch, double Yaw);
		static matd GetTransformMatrix(double Roll, double Pitch, double Yaw, double x, double y, double z);
		static vecd rot2omega(matd Rerr);
		static vecd ConvertRad2Deg(vecd q);
		static vecd GetRPY(matd T);
		static vecd Cross(vecd v1, vecd v2);
	};
}

#endif

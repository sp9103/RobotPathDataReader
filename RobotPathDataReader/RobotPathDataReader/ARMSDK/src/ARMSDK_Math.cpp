#include "ARMSDK_Math.h"

namespace armsdk
{
	matd Algebra::GetOrientationMatrix(double Roll, double Pitch, double Yaw)
	{
		matd R(3,3);

		double sr=sin(Roll);
		double cr=cos(Roll);
		double sp=sin(Pitch);
		double cp=cos(Pitch);
		double sy=sin(Yaw);
		double cy=cos(Yaw);

		R << cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
			sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, 
			-sp,          cp*sr,          cp*cr ;

		return R;
	}
	
	matd Algebra::GetTransformMatrix(double Roll, double Pitch, double Yaw, double x, double y, double z)
	{
		matd T(4,4);
		double sr=sin(Roll);
		double cr=cos(Roll);
		double sp=sin(Pitch);
		double cp=cos(Pitch);
		double sy=sin(Yaw);
		double cy=cos(Yaw);

		T << cy*cp,  cy*sp*sr-sy*cr,  cy*sp*cr+sy*sr,   x,
			sy*cp,  sy*sp*sr+cy*cr,  sy*sp*cr-cy*sr,   y,
			-sp,           cp*sr,           cp*cr,   z,
			0.0,             0.0,             0.0, 1.0; 

		return T;

	}

	vecd Algebra::rot2omega(matd Rerr)
	{
		double alpha = (Rerr(0,0) + Rerr(1,1) + Rerr(2,2) -1)/2;/*(Rerr(0,0) + Rerr(1,1) + Rerr(2,2) - 1)*/
		double th = acos(alpha);

		vecd w(3);

		if (abs(alpha-1) < 0.00000001)
		{
			w(0)=0.0;
			w(1)=0.0;
			w(2)=0.0;
			return w;
		}
		else
		{
			w(0) = 0.5*th/sin(th)*(Rerr(2,1) - Rerr(1,2));
			w(1) = 0.5*th/sin(th)*(Rerr(0,2) - Rerr(2,0));
			w(2) = 0.5*th/sin(th)*(Rerr(1,0) - Rerr(0,1));
			return w;
		}
	}

	vecd Algebra::ConvertRad2Deg(vecd q)
	{
		q = q*_RADtoDEG;
		return q;
	}

	vecd Algebra::GetRPY(matd T)
	{
		vecd rpy(3);
		rpy(0) = atan2( T(2,1), T(2,2)); 
		rpy(1) = atan2(-T(2,0), sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2)) );
		rpy(2) = atan2( T(1,0), T(0,0));

		return rpy;
	}

	vecd Algebra::Cross(vecd v1, vecd v2)
	{
		vecd result(3);
		result(0) = v1(1) * v2(2) - v1(2) * v2(1);
		result(1) = v1(2) * v2(0) - v1(0) * v2(2);
		result(2) = v1(0) * v2(1) - v1(1) * v2(0);

		return result;
	}
}
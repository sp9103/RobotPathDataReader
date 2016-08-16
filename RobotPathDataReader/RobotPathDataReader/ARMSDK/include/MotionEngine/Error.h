#ifndef __ERROR_H_
#define __ERROR_H_

#define ARMSDK_NO_ERROR               0x00
#define ARMSDK_NO_IK_SOLUTION         0x01
#define ARMSDK_ACCEPTABLE_ERROR       0x02  // IK가 풀리진 않으나 Error가 5mm 이하
#define ARMSDK_TOO_MUCH_ANGLE_CHANGE  0x04
#define ARMSDK_OUT_OF_JOINT_RANGE     0x08

inline void ErrorCheck(int Error)
{
	if(Error && ARMSDK_NO_IK_SOLUTION)
		printf("No IK Solution!\n");

	if(Error && ARMSDK_ACCEPTABLE_ERROR)
		printf("But, Result is acceptable(error is not up to 5mm)\n");

	if(Error && ARMSDK_OUT_OF_JOINT_RANGE)
		printf("Result is out of the Joint Limit!\n");

	if(Error && ARMSDK_TOO_MUCH_ANGLE_CHANGE)
		printf("This Step is need too much angle change");
}
#endif
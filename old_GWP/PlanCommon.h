#ifndef _PLANCOMMON_H_
#define _PLANCOMMON_H_

enum FootStatus
{
	BothLeg  = 0,
	RightLeg = 1,
	LeftLeg  = -1,
};

enum WalkingStatus
{
	Start   = 0,
	Walking = 1,
	Stop	= 2,
};

typedef struct
{
	double x;
	double y;
	double th;
}Pos2D;

#endif

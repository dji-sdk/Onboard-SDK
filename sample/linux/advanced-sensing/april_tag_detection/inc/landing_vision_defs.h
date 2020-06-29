/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : landing_vision_defs.h
   Author : tao.jing
   Date   : 19-5-7
   Brief  : 
**************************************************************************/
#ifndef __LANDING_VISION_DEFS__
#define __LANDING_VISION_DEFS__

namespace landing_vision_defs
{
#pragma push(1)
	typedef struct _land_mark_pos
	{
		int mark_id;
		int hammingDistance;
		double distance;
		double x;
		double y;
		double z;
		double yaw;
		double pitch;
		double roll;

	}land_mark_pos;
#pragma pop()
};

#endif

#include "BulletTypes.h"
#include "UnityTypes.h"
#include "LinearMath/btQuickprof.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

typedef void(*EnterProfileZoneEvent)(void* ptr, int len);
typedef void(*LeaveProfileZoneEvent)();

static EnterProfileZoneEvent _enterProfileZone;

static void enterProfileZone(const char* s)
{
	if (_enterProfileZone)
	{
		_enterProfileZone((void*)s, strlen(s));
	}
}

static void defaultEnterProfileZone(const char*)
{
}

static void defaultLeaveProfileZone()
{
}

CLASSFUNC(btCollisionWorld)
STATIC void DLLAPI btCollisionWorld__setProfilerCallbacks_0(void* begin, void* end)
{
#ifndef BT_NO_PROFILE
	_enterProfileZone = (EnterProfileZoneEvent)(begin ? begin : defaultEnterProfileZone);
	btSetCustomEnterProfileZoneFunc(enterProfileZone);
	btSetCustomLeaveProfileZoneFunc((LeaveProfileZoneEvent)(end ? end : defaultLeaveProfileZone));
#endif // BT_NO_PROFILE
}

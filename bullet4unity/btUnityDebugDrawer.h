#ifndef _BTUNITYDEBUGDRAWER_H_
#define _BTUNITYDEBUGDRAWER_H_
#include "BulletTypes.h"
#include "UnityTypes.h"

class btUnityDebugDrawer : public btIDebugDraw
{
public:

	typedef void(*Callback)(Vector3, Vector3, Vector3);

	int _debugMode;
	Callback _callback;

	btUnityDebugDrawer(Callback callback) : _callback(callback), _debugMode(0)
	{

	}

	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		if (_callback)
		{
			_callback(from, to, color);
		}
	}

	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
	{
	}

	virtual void reportErrorWarning(const char* warningString)
	{
	}

	virtual void draw3dText(const btVector3& location, const char* textString)
	{
	}

	virtual void setDebugMode(int debugMode)
	{
		_debugMode = debugMode;
	}

	virtual int getDebugMode() const
	{
		return _debugMode;
	}
};

#endif // _BTUNITYDEBUGDRAWER_H_

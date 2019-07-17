#ifndef _BTMANAGEDCONVEXTRIANGLEMESHSHAPE_H_
#define _BTMANAGEDCONVEXTRIANGLEMESHSHAPE_H_
#include "BulletTypes.h"
#include "btManagedMesh.h"
#include <map>

static std::map<btManagedMesh*, class btManagedConvexTriangleMeshShape*> _managedConvexTriangleMeshShapes;

class btManagedConvexTriangleMeshShape : public btConvexTriangleMeshShape
{
public:

	int m_refCount;

	btManagedConvexTriangleMeshShape(btManagedMesh * meshInterface, bool calcAabb = true)
		: btConvexTriangleMeshShape(meshInterface, calcAabb)
		, m_refCount(0)
	{
		meshInterface->addRefCount();
		_managedConvexTriangleMeshShapes[meshInterface] = this;
	}

	~btManagedConvexTriangleMeshShape()
	{
		auto mesh = (btManagedMesh*)getMeshInterface();
		mesh->release();
		_managedConvexTriangleMeshShapes.erase(mesh);
	}

	int addRefCount()
	{
		++m_refCount;
		return m_refCount;
	}

	int release()
	{
		--m_refCount;
		if (m_refCount == 0)
		{
			delete this;
			return 0;
		}
		return m_refCount;
	}
};

#endif // _BTMANAGEDCONVEXTRIANGLEMESHSHAPE_H_


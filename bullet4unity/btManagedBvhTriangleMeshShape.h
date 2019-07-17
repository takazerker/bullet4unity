#ifndef _BTMANAGEDBVHTRIANGLEMESHSHAPE_H_
#define _BTMANAGEDBVHTRIANGLEMESHSHAPE_H_
#include "BulletTypes.h"
#include "btManagedMesh.h"
#include <map>

static std::map<btManagedMesh*, class btManagedBvhTriangleMeshShape*> _managedBvhTriangleMeshShapes;

class btManagedBvhTriangleMeshShape : public btBvhTriangleMeshShape
{
public:

	int m_refCount;

	btManagedBvhTriangleMeshShape(btManagedMesh* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true)
		: btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, buildBvh)
		, m_refCount(0)
	{
		meshInterface->addRefCount();
		_managedBvhTriangleMeshShapes[meshInterface] = this;
	}

	btManagedBvhTriangleMeshShape(btManagedMesh* meshInterface, bool useQuantizedAabbCompression, const btVector3& bvhAabbMin, const btVector3& bvhAabbMax, bool buildBvh = true)
		: btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax, buildBvh)
	{
		meshInterface->addRefCount();
		_managedBvhTriangleMeshShapes[meshInterface] = this;
	}

	~btManagedBvhTriangleMeshShape()
	{
		btManagedMesh* mesh = (btManagedMesh*)m_meshInterface;
		mesh->release();
		_managedBvhTriangleMeshShapes.erase(mesh);
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

#endif // _BTMANAGEDBVHTRIANGLEMESHSHAPE_H_


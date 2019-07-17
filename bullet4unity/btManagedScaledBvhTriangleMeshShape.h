#ifndef _BTMANAGEDSCALEDBVHTRIANGLEMESHSHAPE_H_
#define _BTMANAGEDSCALEDBVHTRIANGLEMESHSHAPE_H_
#include "BulletTypes.h"
#include "btManagedBvhTriangleMeshShape.h"
#include <map>

class btManagedScaledBvhTriangleMeshShape : public btScaledBvhTriangleMeshShape
{
public:
	btManagedScaledBvhTriangleMeshShape(btManagedBvhTriangleMeshShape * childShape, const btVector3& localScaling)
		: btScaledBvhTriangleMeshShape(childShape, localScaling)
	{
		childShape->addRefCount();
	}

	~btManagedScaledBvhTriangleMeshShape()
	{
		((btManagedBvhTriangleMeshShape*)getChildShape())->release();
	}
};

#endif // _BTMANAGEDSCALEDBVHTRIANGLEMESHSHAPE_H_


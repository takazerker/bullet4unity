#include "BulletTypes.h"
#include "UnityTypes.h"
#include "btManagedMesh.h"
#include "btManagedBvhTriangleMeshShape.h"
#include "btManagedConvexTriangleMeshShape.h"
#include "btManagedScaledBvhTriangleMeshShape.h"

//=============================================================================
// btCollisionShape
//=============================================================================

CLASSFUNC(btCollisionShape)
DESTRUCTOR void DLLAPI btCollisionShape__destroy(btCollisionShape* self)
{
	delete self;
}

CLASSFUNC(btCollisionShape)
void DLLAPI btCollisionShape__getAabb_0(btCollisionShape* self, const btTransform* t, Vector3* aabbMin, Vector3* aabbMax)
{
	self->getAabb(*AlignAddressOf(t), aabbMin->Marshall(), aabbMax->Marshall());
}

CLASSFUNC(btCollisionShape)
void DLLAPI btCollisionShape__getBoundingSphere_0(btCollisionShape* self, Vector3* center, float* radius)
{
	btScalar result;
	self->getBoundingSphere(center->Marshall(), result);
	*radius = result;
}

CLASSFUNC(btCollisionShape)
float DLLAPI btCollisionShape__getAngularMotionDisc_0(btCollisionShape* self)
{
	return self->getAngularMotionDisc();
}

CLASSFUNC(btCollisionShape)
float DLLAPI btCollisionShape__getContactBreakingThreshold_0(btCollisionShape* self, float defaultContactThresholdFactor)
{
	return self->getContactBreakingThreshold(defaultContactThresholdFactor);
}

CLASSFUNC(btCollisionShape)
void DLLAPI btCollisionShape__setLocalScaling_0(btCollisionShape* self, const Vector3* scaling)
{
	self->setLocalScaling(scaling->Marshall());
}

CLASSFUNC(btCollisionShape)
Vector3 DLLAPI btCollisionShape__getLocalScaling_0(btCollisionShape* self)
{
	return Vector3(self->getLocalScaling());
}

CLASSFUNC(btCollisionShape)
void DLLAPI btCollisionShape__calculateLocalInertia_0(btCollisionShape* self, float mass, Vector3* inertia)
{
	self->calculateLocalInertia(mass, inertia->Marshall());
}

CLASSFUNC(btCollisionShape)
void DLLAPI btCollisionShape__calculateTemporalAabb_0(btCollisionShape* self, const btTransform* curTrans, const Vector3* linvel, const Vector3* angvel, float timeStep, Vector3* temporalAabbMin, Vector3* temporalAabbMax)
{
	self->calculateTemporalAabb(*AlignAddressOf(curTrans), linvel->Marshall(), angvel->Marshall(), timeStep, temporalAabbMin->Marshall(), temporalAabbMax->Marshall());
}

CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isPolyhedral_0(btCollisionShape* self)
{
	return self->isPolyhedral();
}

CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isConvex2d_0(btCollisionShape* self)
{
	return self->isConvex2d();
}

CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isConvex_0(btCollisionShape* self)
{
	return self->isConvex();
}
CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isNonMoving_0(btCollisionShape* self)
{
	return self->isNonMoving();
}
CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isConcave_0(btCollisionShape* self)
{
	return self->isConcave();
}
CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isCompound_0(btCollisionShape* self)
{
	return self->isCompound();
}

CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isSoftBody_0(btCollisionShape* self)
{
	return self->isSoftBody();
}

CLASSFUNC(btCollisionShape)
bool DLLAPI btCollisionShape__isInfinite_0(btCollisionShape* self)
{
	return self->isInfinite();
}

CLASSFUNC(btCollisionShape)
void DLLAPI btCollisionShape__setMargin_0(btCollisionShape* self, float margin)
{
	self->setMargin(margin);
}

CLASSFUNC(btCollisionShape)
float DLLAPI btCollisionShape__getMargin_0(btCollisionShape* self)
{
	return self->getMargin();
}

//=============================================================================
// btUniformScalingShape
//=============================================================================

CLASSFUNC(btUniformScalingShape)
STATIC btUniformScalingShape* DLLAPI btUniformScalingShape__create_0(btConvexShape* convexChildShape, float uniformScalingFactor)
{
	return new btUniformScalingShape(convexChildShape, uniformScalingFactor);
}

//=============================================================================
// btConvexShape
//=============================================================================

CLASSFUNC(btConvexShape)
int DLLAPI btConvexShape__getNumPreferredPenetrationDirections_0(btConvexShape* self)
{
	return self->getNumPreferredPenetrationDirections();
}

CLASSFUNC(btConvexShape)
void DLLAPI btConvexShape__getPreferredPenetrationDirection(btConvexShape* self, int index, Vector3* penetrationVector)
{
	return self->getPreferredPenetrationDirection(index, penetrationVector->Marshall());
}

//=============================================================================
// btStaticPlaneShape
//=============================================================================

CLASSFUNC(btStaticPlaneShape)
STATIC btStaticPlaneShape* DLLAPI btStaticPlaneShape__create_0(const Vector3* planeNormal, float planeConstant)
{
	return new btStaticPlaneShape(planeNormal->Marshall(), planeConstant);
}

//=============================================================================
// btBoxShape
//=============================================================================

CLASSFUNC(btBoxShape)
STATIC btBoxShape* DLLAPI btBoxShape__create_0(const Vector3* boxHalfExtents)
{
	return new btBoxShape(boxHalfExtents->Marshall());
}

CLASSFUNC(btBoxShape)
Vector3 DLLAPI btBoxShape__getHalfExtentsWithMargin_0(btBoxShape* self)
{
	return self->getHalfExtentsWithMargin();
}

CLASSFUNC(btBoxShape)
Vector3 DLLAPI btBoxShape__getHalfExtentsWithoutMargin_0(btBoxShape* self)
{
	return self->getHalfExtentsWithoutMargin();
}

//=============================================================================
// btSphereShape
//=============================================================================

CLASSFUNC(btSphereShape)
STATIC btSphereShape* DLLAPI btSphereShape__create_0(float radius)
{
	return new btSphereShape(radius);
}

//=============================================================================
// btCylinderShape
//=============================================================================

CLASSFUNC(btCylinderShape)
STATIC btCylinderShape* DLLAPI btCylinderShape__create_0(const Vector3* halfExtents)
{
	return new btCylinderShape(halfExtents->Marshall());
}

CLASSFUNC(btCylinderShapeX)
STATIC btCylinderShapeX* DLLAPI btCylinderShapeX__create_0(const Vector3* halfExtents)
{
	return new btCylinderShapeX(halfExtents->Marshall());
}

CLASSFUNC(btCylinderShapeZ)
STATIC btCylinderShapeZ* DLLAPI btCylinderShapeZ__create_0(const Vector3* halfExtents)
{
	return new btCylinderShapeZ(halfExtents->Marshall());
}

//=============================================================================
// btCapsuleShape
//=============================================================================

CLASSFUNC(btCapsuleShape)
STATIC btCapsuleShape* DLLAPI btCapsuleShape__create_0(float radius, float height)
{
	return new btCapsuleShape(radius, height);
}

CLASSFUNC(btCapsuleShapeX)
STATIC btCapsuleShapeX* DLLAPI btCapsuleShapeX__create_0(float radius, float height)
{
	return new btCapsuleShapeX(radius, height);
}

CLASSFUNC(btCapsuleShapeZ)
STATIC btCapsuleShapeZ* DLLAPI btCapsuleShapeZ__create_0(float radius, float height)
{
	return new btCapsuleShapeZ(radius, height);
}

//=============================================================================
// btConeShape
//=============================================================================

CLASSFUNC(btConeShape)
STATIC btConeShape* DLLAPI btConeShape__create_0(float radius, float height)
{
	return new btConeShape(radius, height);
}

CLASSFUNC(btConeShapeX)
STATIC btConeShapeX* DLLAPI btConeShapeX__create_0(float radius, float height)
{
	return new btConeShapeX(radius, height);
}

CLASSFUNC(btConeShapeZ)
STATIC btConeShapeZ* DLLAPI btConeShapeZ__create_0(float radius, float height)
{
	return new btConeShapeZ(radius, height);
}

//=============================================================================
// btConvexInternalShape
//=============================================================================

CLASSFUNC(btConvexInternalShape)
void DLLAPI btConvexInternalShape__setImplicitShapeDimensions(btConvexInternalShape* self, const Vector3* vec)
{
	self->setImplicitShapeDimensions(vec->Marshall());
}

CLASSFUNC(btConvexInternalShape)
Vector3 DLLAPI btConvexInternalShape__getImplicitShapeDimensions(btConvexInternalShape* self)
{
	return self->getImplicitShapeDimensions();
}

//=============================================================================
// btConvexHullShape
//=============================================================================

CLASSFUNC(btConvexHullShape)
STATIC btConvexHullShape* DLLAPI btConvexHullShape__create_0()
{
	return new btConvexHullShape();
}

CLASSFUNC(btConvexHullShape)
void DLLAPI btConvexHullShape__addPoint_0(btConvexHullShape* self, const Vector3* point, bool recalculateLocalAabb = true)
{
	self->addPoint(point->Marshall(), recalculateLocalAabb);
}

CLASSFUNC(btConvexHullShape)
void DLLAPI btConvexHullShape__optimizeConvexHull_0(btConvexHullShape* self)
{
	self->optimizeConvexHull();
}

//=============================================================================
// btCompoundShape
//=============================================================================

CLASSFUNC(btCompoundShape)
STATIC btCompoundShape* DLLAPI btCompoundShape__create_0(bool enableDynamicAabbTree = true, int initialChildCapacity = 0)
{
	return new btCompoundShape(enableDynamicAabbTree, initialChildCapacity);
}

CLASSFUNC(btCompoundShape)
int DLLAPI btCompoundShape__findChildShape_0(btCompoundShape* self, btCollisionShape* shape)
{
	for (int i = 0; i < self->getNumChildShapes(); ++i)
	{
		if (self->getChildShape(i) == shape)
		{
			return i;
		}
	}
	return -1;
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__addChildShape_0(btCompoundShape* self, const btTransform* localTransform, btCollisionShape* shape)
{
	self->addChildShape(*AlignAddressOf(localTransform), shape);
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__removeChildShape_0(btCompoundShape* self, btCollisionShape* shape)
{
	self->removeChildShape(shape);
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__removeChildShapeByIndex_0(btCompoundShape* self, int childShapeIndex)
{
	self->removeChildShapeByIndex(childShapeIndex);
}

CLASSFUNC(btCompoundShape)
int DLLAPI btCompoundShape__getNumChildShapes_0(btCompoundShape* self)
{
	return self->getNumChildShapes();
}

CLASSFUNC(btCompoundShape)
btCollisionShape* DLLAPI btCompoundShape__getChildShape(btCompoundShape* self, int index)
{
	return self->getChildShape(index);
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__getChildTransform_0(btCompoundShape* self, int index, btTransform* localTransform)
{
	*AlignAddressOf(localTransform) = self->getChildTransform(index);
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__updateChildTransform_0(btCompoundShape* self, int childIndex, const btTransform* newChildTransform, bool shouldRecalculateLocalAabb = true)
{
	self->updateChildTransform(childIndex, *AlignAddressOf(newChildTransform), shouldRecalculateLocalAabb);
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__createAabbTreeFromChildren_0(btCompoundShape* self)
{
	self->createAabbTreeFromChildren();
}

CLASSFUNC(btCompoundShape)
void DLLAPI btCompoundShape__recalculateLocalAabb_0(btCompoundShape* self)
{
	self->recalculateLocalAabb();
}

//=============================================================================
// btScaledBvhTriangleMeshShape
//=============================================================================

CLASSFUNC(btScaledBvhTriangleMeshShape)
STATIC btScaledBvhTriangleMeshShape* DLLAPI btScaledBvhTriangleMeshShape__create_0(btBvhTriangleMeshShape* childShape, const Vector3* localScaling)
{
	return new btScaledBvhTriangleMeshShape(childShape, localScaling->Marshall());
}

//=============================================================================
// btBvhTriangleMeshShape
//=============================================================================

CLASSFUNC(btBvhTriangleMeshShape)
STATIC btBvhTriangleMeshShape* DLLAPI btBvhTriangleMeshShape__create_0(btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true)
{
	return new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, buildBvh);
}

//=============================================================================
// btHeightfieldTerrainShape
//=============================================================================

CLASSFUNC(btHeightfieldTerrainShape)
STATIC btHeightfieldTerrainShape* DLLAPI btHeightfieldTerrainShape__create_0(
	int heightStickWidth, int heightStickLength,
	void* heightfieldData, float heightScale,
	float minHeight, float maxHeight,
	int upAxis, PHY_ScalarType heightDataType,
	bool flipQuadEdges)
{
	return new btHeightfieldTerrainShape(heightStickWidth, heightStickLength,
		heightfieldData, heightScale,
		minHeight, maxHeight,
		upAxis, heightDataType,
		flipQuadEdges);
}

//=============================================================================
// btManagedBvhTriangleMeshShape
//=============================================================================

CLASSFUNC(btManagedBvhTriangleMeshShape)
STATIC btManagedBvhTriangleMeshShape* DLLAPI btManagedBvhTriangleMeshShape__create_0(btManagedMesh* meshInterface)
{
	btAssert(meshInterface);

	auto itr = _managedBvhTriangleMeshShapes.find(meshInterface);
	btManagedBvhTriangleMeshShape* result = 0;

	if (itr != _managedBvhTriangleMeshShapes.end())
	{
		result = itr->second;
	}
	else
	{
		result = new btManagedBvhTriangleMeshShape(meshInterface, true, true);
	}

	result->addRefCount();
	return result;
}

CLASSFUNC(btManagedBvhTriangleMeshShape)
int DLLAPI btManagedBvhTriangleMeshShape__addRefCount_0(btManagedBvhTriangleMeshShape* self)
{
	return self->addRefCount();
}

CLASSFUNC(btManagedBvhTriangleMeshShape)
int DLLAPI btManagedBvhTriangleMeshShape__release_0(btManagedBvhTriangleMeshShape* self)
{
	return self->release();
}

//=============================================================================
// btManagedConvexTriangleMeshShape
//=============================================================================

CLASSFUNC(btManagedConvexTriangleMeshShape)
STATIC btManagedConvexTriangleMeshShape* DLLAPI btManagedConvexTriangleMeshShape__create_0(btManagedMesh* meshInterface)
{
	btAssert(meshInterface);

	auto itr = _managedConvexTriangleMeshShapes.find(meshInterface);
	btManagedConvexTriangleMeshShape* result = 0;

	if (itr != _managedConvexTriangleMeshShapes.end())
	{
		result = itr->second;
	}
	else
	{
		result = new btManagedConvexTriangleMeshShape(meshInterface, true);
	}

	result->addRefCount();
	return result;
}

CLASSFUNC(btManagedConvexTriangleMeshShape)
int DLLAPI btManagedConvexTriangleMeshShape__addRefCount_0(btManagedConvexTriangleMeshShape* self)
{
	return self->addRefCount();
}

CLASSFUNC(btManagedConvexTriangleMeshShape)
int DLLAPI btManagedConvexTriangleMeshShape__release_0(btManagedConvexTriangleMeshShape* self)
{
	return self->release();
}

//=============================================================================
// btManagedConvexTriangleMeshShape
//=============================================================================

CLASSFUNC(btManagedScaledBvhTriangleMeshShape)
STATIC btManagedScaledBvhTriangleMeshShape* DLLAPI btManagedScaledBvhTriangleMeshShape__create_0(btManagedBvhTriangleMeshShape * childShape, const Vector3* localScaling)
{
	auto result = new btManagedScaledBvhTriangleMeshShape(childShape, localScaling->Marshall());
	return result;
}


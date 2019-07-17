#include "BulletTypes.h"
#include "UnityTypes.h"
#include "btUnityDebugDrawer.h"
#include "btFilteredConvexResultCallback.h"

EXPORT_STRUCT(btCollisionData);
EXPORT_STRUCT(btDefaultCollisionConstructionInfo);

//=============================================================================
// btCollisionWorld
//=============================================================================

CLASSFUNC(btCollisionWorld)
STATIC void DLLAPI btCollisionWorld__setDeactivationTime_0(float time)
{
	gDeactivationTime = time;
}

CLASSFUNC(btCollisionWorld)
DESTRUCTOR void DLLAPI btCollisionWorld__destroy(btCollisionWorld* self)
{
	delete self;
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__setBroadphase_0(btCollisionWorld* self, btBroadphaseInterface* pairCache)
{
	self->setBroadphase(pairCache);
}

CLASSFUNC(btCollisionWorld)
btBroadphaseInterface* DLLAPI btCollisionWorld__getBroadphase_0(btCollisionWorld* self)
{
	return self->getBroadphase();
}

CLASSFUNC(btCollisionWorld)
btOverlappingPairCache* DLLAPI btCollisionWorld__getPairCache_0(btCollisionWorld* self)
{
	return self->getPairCache();
}

CLASSFUNC(btCollisionWorld)
btDispatcher* DLLAPI btCollisionWorld__getDispatcher_0(btCollisionWorld* self)
{
	return self->getDispatcher();
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__updateSingleAabb_0(btCollisionWorld* self, btCollisionObject* colObj)
{
	self->updateSingleAabb(colObj);
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__updateAabbs_0(btCollisionWorld* self)
{
	self->updateAabbs();
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__computeOverlappingPairs_0(btCollisionWorld* self)
{
	self->computeOverlappingPairs();
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__setDebugDrawer_0(btCollisionWorld* self, btIDebugDraw* debugDrawer)
{
	self->setDebugDrawer(debugDrawer);
}

CLASSFUNC(btCollisionWorld)
btIDebugDraw* DLLAPI btCollisionWorld__getDebugDrawer_0(btCollisionWorld* self)
{
	return self->getDebugDrawer();
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__debugDrawWorld_0(btCollisionWorld* self)
{
	self->debugDrawWorld();
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__debugDrawObject_0(btCollisionWorld* self, const btTransform* worldTransform, btCollisionShape* shape, const Vector4* color)
{
	self->debugDrawObject(*AlignAddressOf(worldTransform), shape, color->Marshall());
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__rayTest_0(btCollisionWorld* self, const Vector3* rayFromWorld, const Vector3* rayToWorld, btCollisionWorld::RayResultCallback* resultCallback)
{
	self->rayTest(rayFromWorld->Marshall(), rayToWorld->Marshall(), *resultCallback);
}

CLASSFUNC(btCollisionWorld)
bool DLLAPI btCollisionWorld__rayTest_1(btCollisionWorld* self, const Vector3* rayFromWorld, const Vector3* rayToWorld, btCollisionData* result, int filterGroup, int filterMask)
{
	btCollisionWorld::ClosestRayResultCallback cb(rayFromWorld->Marshall(), rayToWorld->Marshall());
	cb.m_collisionFilterGroup = filterGroup;
	cb.m_collisionFilterMask = filterMask;

	self->rayTest(rayFromWorld->Marshall(), rayToWorld->Marshall(), cb);

	memset(result, 0, sizeof(*result));
	result->fraction = cb.m_closestHitFraction;
	result->collisionObject = (btCollisionObject*)cb.m_collisionObject;
	result->normalWorld = cb.m_hitNormalWorld;
	result->pointWorld = cb.m_hitPointWorld;
	return cb.hasHit();
}

CLASSFUNC(btCollisionWorld)
void DLLAPI btCollisionWorld__convexSweepTest_0(btCollisionWorld* self, btConvexShape* castShape, const btTransform* from, const btTransform* to, btCollisionWorld::ConvexResultCallback* resultCallback, float allowedCcdPenetration = btScalar(0.))
{
	self->convexSweepTest(castShape, *AlignAddressOf(from), *AlignAddressOf(to), *resultCallback, allowedCcdPenetration);
}

CLASSFUNC(btCollisionWorld)
int DLLAPI btCollisionWorld__contactTest_0(btCollisionWorld* self, btPairTestCache* cache, btCollisionObject* colObj, int filterGroup, int filterMask)
{
	struct ContactCallback : btCollisionWorld::ContactResultCallback
	{
		btPairTestCache* cache;
		btCollisionObject* colObj;
		btIDebugDraw* debugDraw;

		virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
		{
			bool swapped = colObj0Wrap->m_collisionObject != colObj;

			if (cp.m_distance1 <= 0)
			{
				cache->objects.push_back(const_cast<btCollisionObject*>(swapped ? colObj0Wrap->m_collisionObject : colObj1Wrap->m_collisionObject));
			}

			return 0;
		}
	};

	cache->objects.clear();

	ContactCallback cb;
	cb.colObj = colObj;
	cb.debugDraw = self->getDebugDrawer();
	cb.m_collisionFilterGroup = filterGroup;
	cb.m_collisionFilterMask = filterMask;
	cb.cache = cache;

	self->contactTest(colObj, cb);

	return cache->objects.size();
}

//=============================================================================
// btPairTestCache
//=============================================================================
CLASSFUNC(btPairTestCache)
STATIC btPairTestCache* DLLAPI btPairTestCache__create_0()
{
	return new btPairTestCache();
}

CLASSFUNC(btPairTestCache)
DESTRUCTOR void DLLAPI btPairTestCache__destroy(btPairTestCache* self)
{
	delete self;
}

CLASSFUNC(btPairTestCache)
btCollisionObject* DLLAPI btPairTestCache__getCollisionObject_0(btPairTestCache* self, int index)
{
	return self->objects[index];
}

CLASSFUNC(btPairTestCache)
int DLLAPI btPairTestCache__getNumObjects_0(btPairTestCache* self)
{
	return self->objects.size();
}

//=============================================================================
// btDynamicsWorld
//=============================================================================

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__stepSimulation_0(btDynamicsWorld* self, float timeStep, int maxSubSteps = 1, float fixedTimeStep = 1.0f / 60.0f)
{
	self->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addConstraint_0(btDynamicsWorld* self, btTypedConstraint *constraint, bool disableCollisionsBetweenLinkedBodies = false)
{
	self->addConstraint(constraint, disableCollisionsBetweenLinkedBodies);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__removeConstraint_0(btDynamicsWorld* self, btTypedConstraint *constraint)
{
	self->removeConstraint(constraint);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addAction_0(btDynamicsWorld* self, btActionInterface *action)
{
	self->addAction(action);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__removeAction_0(btDynamicsWorld* self, btActionInterface *action)
{
	self->removeAction(action);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__setGravity_0(btDynamicsWorld* self, const Vector3* gravity)
{
	self->setGravity(gravity->Marshall());
}

CLASSFUNC(btDynamicsWorld)
Vector3 DLLAPI btDynamicsWorld__getGravity_0(btDynamicsWorld* self)
{
	return self->getGravity();
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__synchronizeMotionStates_0(btDynamicsWorld* self)
{
	self->synchronizeMotionStates();
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addRigidBody_0(btDynamicsWorld* self, btRigidBody* body)
{
	self->addRigidBody(body);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addRigidBody_1(btDynamicsWorld* self, btRigidBody* body, int group, int mask)
{
	self->addRigidBody(body, group, mask);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__removeRigidBody_0(btDynamicsWorld* self, btRigidBody* body)
{
	self->removeRigidBody(body);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__setConstraintSolver_0(btDynamicsWorld* self, btConstraintSolver *solver)
{
	self->setConstraintSolver(solver);
}

CLASSFUNC(btDynamicsWorld)
btConstraintSolver* DLLAPI btDynamicsWorld__getConstraintSolver_0(btDynamicsWorld* self)
{
	return self->getConstraintSolver();
}

CLASSFUNC(btDynamicsWorld)
int DLLAPI btDynamicsWorld__getNumConstraints_0(btDynamicsWorld* self)
{
	return self->getNumConstraints();
}

CLASSFUNC(btDynamicsWorld)
btTypedConstraint* DLLAPI btDynamicsWorld__getConstraint_0(btDynamicsWorld* self, int index)
{
	return self->getConstraint(index);
}

CLASSFUNC(btDynamicsWorld)
btDynamicsWorldType DLLAPI btDynamicsWorld__getWorldType_0(btDynamicsWorld* self)
{
	return self->getWorldType();
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__clearForces_0(btDynamicsWorld* self)
{
	self->clearForces();
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__setWorldUserInfo_0(btDynamicsWorld* self, void* worldUserInfo)
{
	self->setWorldUserInfo(worldUserInfo);
}

CLASSFUNC(btDynamicsWorld)
void* DLLAPI btDynamicsWorld__getWorldUserInfo_0(btDynamicsWorld* self)
{
	return self->getWorldUserInfo();
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__setInternalTickCallback_0(btDynamicsWorld* self, void* cb, void *worldUserInfo = 0, bool isPreTick = false)
{
	self->setInternalTickCallback((btInternalTickCallback)cb, worldUserInfo, isPreTick);
}

//CLASSFUNC(btDynamicsWorld)
//btContactSolverInfo* DLLAPI btDynamicsWorld__getSolverInfo(btDynamicsWorld* self)
//{
//	return self->getSolverInfo();
//}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addVehicle(btDynamicsWorld* self, btActionInterface *vehicle)
{
	self->addVehicle(vehicle);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__removeVehicle(btDynamicsWorld* self, btActionInterface *vehicle)
{
	self->removeVehicle(vehicle);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addCharacter(btDynamicsWorld* self, btActionInterface *Character)
{
	self->addCharacter(Character);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__removeCharacter(btDynamicsWorld* self, btActionInterface *Character)
{
	self->removeCharacter(Character);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__addCollisionObject_0(btDynamicsWorld* self, btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
{
	self->addCollisionObject(collisionObject, collisionFilterGroup, collisionFilterMask);
}

CLASSFUNC(btDynamicsWorld)
void DLLAPI btDynamicsWorld__removeCollisionObject_0(btDynamicsWorld* self, btCollisionObject* collisionObject)
{
	self->removeCollisionObject(collisionObject);
}

//=============================================================================
// btDefaultCollisionConfiguration
//=============================================================================
CLASSFUNC(btPoolAllocator)
DESTRUCTOR void DLLAPI btPoolAllocator__destroy(btPoolAllocator* self)
{
	delete self;
}

//=============================================================================
// btDefaultCollisionConfiguration
//=============================================================================

CLASSFUNC(btDefaultCollisionConfiguration)
STATIC btDefaultCollisionConfiguration* DLLAPI btDefaultCollisionConfiguration__create_0(const btDefaultCollisionConstructionInfo* constructionInfo)
{
	return new btDefaultCollisionConfiguration(*constructionInfo);
}

CLASSFUNC(btDefaultCollisionConfiguration)
DESTRUCTOR void DLLAPI btDefaultCollisionConfiguration__destroy(btDefaultCollisionConfiguration* self)
{
	delete self;
}

//=============================================================================
// btCollisionDispatcher
//=============================================================================

CLASSFUNC(btCollisionDispatcher)
STATIC btCollisionDispatcher* DLLAPI btCollisionDispatcher__create_0(btCollisionConfiguration* collisionConfiguration)
{
	return new btCollisionDispatcher(collisionConfiguration);
}

CLASSFUNC(btCollisionDispatcher)
DESTRUCTOR void DLLAPI btCollisionDispatcher__destroy(btCollisionDispatcher* self)
{
	delete self;
}

//=============================================================================
// btBroadphasePair
//=============================================================================

CLASSFUNC(btBroadphasePair)
btBroadphaseProxy* DLLAPI btBroadphasePair__getProxy0(btBroadphasePair* self)
{
	return self->m_pProxy0;
}

CLASSFUNC(btBroadphasePair)
btBroadphaseProxy* DLLAPI btBroadphasePair__getProxy1(btBroadphasePair* self)
{
	return self->m_pProxy1;
}

//=============================================================================
// btOverlappingPairCallback
//=============================================================================

CLASSFUNC(btOverlappingPairCallback)
btBroadphasePair* DLLAPI btOverlappingPairCallback__addOverlappingPair(btOverlappingPairCallback* self, btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
{
	return self->addOverlappingPair(proxy0, proxy1);
}

CLASSFUNC(btOverlappingPairCallback)
void* DLLAPI btOverlappingPairCallback__removeOverlappingPair(btOverlappingPairCallback* self, btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, btDispatcher* dispatcher)
{
	return self->removeOverlappingPair(proxy0, proxy1, dispatcher);
}

CLASSFUNC(btOverlappingPairCallback)
void DLLAPI btOverlappingPairCallback__removeOverlappingPairsContainingProxy(btOverlappingPairCallback* self, btBroadphaseProxy* proxy0, btDispatcher* dispatcher)
{
	self->removeOverlappingPairsContainingProxy(proxy0, dispatcher);
}

//=============================================================================
// btNullPairCache
//=============================================================================

CLASSFUNC(btNullPairCache)
STATIC btNullPairCache* DLLAPI btNullPairCache__create_0()
{
	return new btNullPairCache();
}

//=============================================================================
// btOverlappingPairCache
//=============================================================================

CLASSFUNC(btOverlappingPairCache)
void DLLAPI btOverlappingPairCache__setInternalGhostPairCallback_0(btOverlappingPairCache* self, btGhostPairCallback* ghostPairCallback)
{
	self->setInternalGhostPairCallback(ghostPairCallback);
}

//=============================================================================
// btHashedOverlappingPairCache
//=============================================================================

CLASSFUNC(btHashedOverlappingPairCache)
STATIC btHashedOverlappingPairCache* DLLAPI btHashedOverlappingPairCache__create_0()
{
	return new btHashedOverlappingPairCache();
}

//=============================================================================
// btSortedOverlappingPairCache
//=============================================================================

CLASSFUNC(btSortedOverlappingPairCache)
STATIC btSortedOverlappingPairCache* DLLAPI btSortedOverlappingPairCache__create_0()
{
	return new btSortedOverlappingPairCache();
}

//=============================================================================
// btBroadphaseInterface
//=============================================================================

CLASSFUNC(btBroadphaseInterface)
DESTRUCTOR void DLLAPI btBroadphaseInterface__destroy(btBroadphaseInterface* self)
{
	delete self;
}

CLASSFUNC(btBroadphaseInterface)
btOverlappingPairCache* DLLAPI btBroadphaseInterface__getOverlappingPairCache_0(btBroadphaseInterface* self)
{
	return self->getOverlappingPairCache();
}

//=============================================================================
// btDbvtBroadphase
//=============================================================================

CLASSFUNC(btDbvtBroadphase)
STATIC btDbvtBroadphase* DLLAPI btDbvtBroadphase__create_0(btOverlappingPairCache* paircache)
{
	return new btDbvtBroadphase(paircache);
}

//=============================================================================
// btSimpleBroadphase
//=============================================================================

CLASSFUNC(btSimpleBroadphase)
STATIC btSimpleBroadphase* DLLAPI btSimpleBroadphase__create_0(int maxProxies, btOverlappingPairCache* paircache)
{
	return new btSimpleBroadphase(maxProxies, paircache);
}

//=============================================================================
// bt32BitAxisSweep3
//=============================================================================

CLASSFUNC(bt32BitAxisSweep3)
bt32BitAxisSweep3* DLLAPI bt32BitAxisSweep3__create_0(Vector3* worldAabbMin, Vector3* worldAabbMax, uint maxHandles = 1500000, btOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false)
{
	return new bt32BitAxisSweep3(worldAabbMin->Marshall(), worldAabbMax->Marshall(), maxHandles, pairCache, disableRaycastAccelerator);
}

//=============================================================================
// btAxisSweep3
//=============================================================================

CLASSFUNC(btAxisSweep3)
STATIC btAxisSweep3* DLLAPI btAxisSweep3__create_0(Vector3* worldAabbMin, Vector3* worldAabbMax, uint maxHandles = 1500000, btOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false)
{
	return new btAxisSweep3(worldAabbMin->Marshall(), worldAabbMax->Marshall(), maxHandles, pairCache, disableRaycastAccelerator);
}

//=============================================================================
// btConstraintSolver
//=============================================================================

CLASSFUNC(btConstraintSolver)
DESTRUCTOR void DLLAPI btConstraintSolver__destroy(btConstraintSolver* self)
{
	delete self;
}

//=============================================================================
// btSequentialImpulseConstraintSolver
//=============================================================================

CLASSFUNC(btSequentialImpulseConstraintSolver)
STATIC btSequentialImpulseConstraintSolver* DLLAPI btSequentialImpulseConstraintSolver__create_0()
{
	return new btSequentialImpulseConstraintSolver();
}

//=============================================================================
// btDiscreteDynamicsWorld
//=============================================================================

CLASSFUNC(btDiscreteDynamicsWorld)
STATIC btDiscreteDynamicsWorld* DLLAPI btDiscreteDynamicsWorld__create_0(btDispatcher * dispatcher, btBroadphaseInterface * pairCache, btConstraintSolver * constraintSolver, btCollisionConfiguration * collisionConfiguration)
{
	return new btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration);
}

//=============================================================================
// btBroadphaseProxy
//=============================================================================

CLASSFUNC(btBroadphaseProxy)
void DLLAPI btBroadphaseProxy__setCollisionFilterGroup_0(btBroadphaseProxy* self, int group)
{
	self->m_collisionFilterGroup = group;
}

CLASSFUNC(btBroadphaseProxy)
void DLLAPI btBroadphaseProxy__setCollisionFilterMask_0(btBroadphaseProxy* self, int mask)
{
	self->m_collisionFilterMask = mask;
}

CLASSFUNC(btBroadphaseProxy)
int DLLAPI btBroadphaseProxy__getCollisionFilterGroup_0(btBroadphaseProxy* self)
{
	return self->m_collisionFilterGroup;
}

CLASSFUNC(btBroadphaseProxy)
int DLLAPI btBroadphaseProxy__getCollisionFilterMask_0(btBroadphaseProxy* self)
{
	return self->m_collisionFilterMask;
}

//=============================================================================
// btCollisionWorld::RayResultCallback
//=============================================================================

CLASSFUNC(btCollisionWorld::RayResultCallback)
DESTRUCTOR void DLLAPI btCollisionWorld__RayResultCallback__destroy(btCollisionWorld::RayResultCallback* self)
{
	delete self;
}

//=============================================================================
// btCollisionWorld::ClosestRayResultCallback
//=============================================================================

CLASSFUNC(btCollisionWorld::ClosestRayResultCallback)
STATIC btCollisionWorld::ClosestRayResultCallback* DLLAPI btCollisionWorld__ClosestRayResultCallback__create_0(const Vector3* from, const Vector3* to, int filterGroup, int filterMask)
{
	auto result = new btCollisionWorld::ClosestRayResultCallback(from->Marshall(), to->Marshall());
	result->m_collisionFilterGroup = filterGroup;
	result->m_collisionFilterMask = filterMask;
	return result;
}

CLASSFUNC(btCollisionWorld::ClosestRayResultCallback)
void DLLAPI btCollisionWorld__ClosestRayResultCallback__getResult_0(btCollisionWorld::ClosestRayResultCallback* self, btCollisionData* result)
{
	memset(result, 0, sizeof(*result));
	result->fraction = self->m_closestHitFraction;
	result->collisionObject = (btCollisionObject*)self->m_collisionObject;
	result->normalWorld = self->m_hitNormalWorld;
	result->pointWorld = self->m_hitPointWorld;
}

//=============================================================================
// btCollisionWorld::AllHitsRayResultCallback
//=============================================================================

CLASSFUNC(btCollisionWorld::AllHitsRayResultCallback)
STATIC btCollisionWorld::AllHitsRayResultCallback* DLLAPI btCollisionWorld__AllHitsRayResultCallback__create_0(const Vector3* from, const Vector3* to, int filterGroup, int filterMask)
{
	auto result = new btCollisionWorld::AllHitsRayResultCallback(from->Marshall(), to->Marshall());
	result->m_collisionFilterGroup = filterGroup;
	result->m_collisionFilterMask = filterMask;
	return result;
}

CLASSFUNC(btCollisionWorld::AllHitsRayResultCallback)
void DLLAPI btCollisionWorld__AllHitsRayResultCallback__reset_0(btCollisionWorld::AllHitsRayResultCallback* self, const Vector3* from, const Vector3* to, int filterGroup, int filterMask)
{
	self->m_rayFromWorld = from->Marshall();
	self->m_rayToWorld = to->Marshall();
	self->m_collisionFilterGroup = filterGroup;
	self->m_collisionFilterMask = filterMask;
	self->m_collisionObjects.clear();
	self->m_hitNormalWorld.clear();
	self->m_hitPointWorld.clear();
	self->m_hitFractions.clear();
}

CLASSFUNC(btCollisionWorld::AllHitsRayResultCallback)
void DLLAPI btCollisionWorld__AllHitsRayResultCallback__getResult_0(btCollisionWorld::AllHitsRayResultCallback* self, btCollisionData* result, int index)
{
	memset(result, 0, sizeof(*result));
	if ((uint)index < (uint)self->m_hitFractions.size())
	{
		result->fraction = self->m_hitFractions[index];
		result->collisionObject = (btCollisionObject*)self->m_collisionObjects[index];
		result->normalWorld = self->m_hitNormalWorld[index];
		result->pointWorld = self->m_hitPointWorld[index];
	}
}

CLASSFUNC(btCollisionWorld::AllHitsRayResultCallback)
int DLLAPI btCollisionWorld__AllHitsRayResultCallback__getResultCount_0(btCollisionWorld::AllHitsRayResultCallback* self)
{
	return (int)self->m_hitFractions.size();
}

//=============================================================================
// btCollisionWorld::ConvexResultCallback
//=============================================================================
CLASSFUNC(btCollisionWorld::ConvexResultCallback)
DESTRUCTOR void DLLAPI btCollisionWorld__ConvexResultCallback__destroy(btCollisionWorld::ConvexResultCallback* self)
{
	delete self;
}

//=============================================================================
// btCollisionWorld::ClosestConvexResultCallback
//=============================================================================

CLASSFUNC(btCollisionWorld::ClosestConvexResultCallback)
STATIC btCollisionWorld::ClosestConvexResultCallback* DLLAPI btCollisionWorld__ClosestConvexResultCallback__create_0(const Vector3* from, const Vector3* to, int filterGroup, int filterMask)
{
	auto result = new btCollisionWorld::ClosestConvexResultCallback(from->Marshall(), to->Marshall());
	result->m_collisionFilterGroup = filterGroup;
	result->m_collisionFilterMask = filterMask;
	return result;
}

CLASSFUNC(btCollisionWorld::ClosestConvexResultCallback)
void DLLAPI btCollisionWorld__ClosestConvexResultCallback__getResult_0(btCollisionWorld::ClosestConvexResultCallback* self, btCollisionData* result)
{
	memset(result, 0, sizeof(*result));
	result->fraction = self->m_closestHitFraction;
	result->collisionObject = (btCollisionObject*)self->m_hitCollisionObject;
	result->normalWorld = self->m_hitNormalWorld;
	result->pointWorld = self->m_hitPointWorld;
}

//=============================================================================
// btFilteredConvexResultCallback
//=============================================================================

CLASSFUNC(btFilteredConvexResultCallback)
STATIC btFilteredConvexResultCallback* DLLAPI btFilteredConvexResultCallback__create_0(const Vector3* from, const Vector3* to, int filterGroup, int filterMask, void* ignoreList, uint ignoreListSize, bool all)
{
	auto result = new btFilteredConvexResultCallback(from->Marshall(), to->Marshall(), reinterpret_cast<btCollisionObject**>(ignoreList), ignoreListSize, all);
	result->m_collisionFilterGroup = filterGroup;
	result->m_collisionFilterMask = filterMask;
	return result;
}

CLASSFUNC(btFilteredConvexResultCallback)
int DLLAPI btFilteredConvexResultCallback__getNumHits_0(btFilteredConvexResultCallback* self)
{
	return self->getNumHits();
}

CLASSFUNC(btFilteredConvexResultCallback)
void DLLAPI btFilteredConvexResultCallback__getResult_0(btFilteredConvexResultCallback* self, int index, btCollisionData* result)
{
	*result = self->getResult(index);
}

//=============================================================================
// btCollisionWorld::ContactResultCallback
//=============================================================================

CLASSFUNC(btCollisionWorld::ContactResultCallback)
DESTRUCTOR void DLLAPI btCollisionWorld__ContactResultCallback__destroy(btCollisionWorld::ContactResultCallback* self)
{
	delete self;
}

//=============================================================================
// btIDebugDraw
//=============================================================================

CLASSFUNC(btIDebugDraw)
DESTRUCTOR void DLLAPI btIDebugDraw__destroy(btIDebugDraw* self)
{
	delete self;
}

CLASSFUNC(btIDebugDraw)
void DLLAPI btIDebugDraw__setDebugMode(btIDebugDraw* self, DebugDrawFlags mode)
{
	self->setDebugMode(mode);
}

//=============================================================================
// btUnityDebugDrawer
//=============================================================================

CLASSFUNC(btUnityDebugDrawer)
STATIC btUnityDebugDrawer* DLLAPI btUnityDebugDrawer__create_0(void* callback)
{
	return new btUnityDebugDrawer((btUnityDebugDrawer::Callback)callback);
}

//=============================================================================
// btCollisionAlgorithm
//=============================================================================

CLASSFUNC(btCollisionAlgorithm)
DESTRUCTOR void DLLAPI btCollisionAlgorithm__destroy(btCollisionAlgorithm* self)
{
	delete self;
}

#if 0
CLASSFUNC(btCollisionAlgorithm)
void DLLAPI btCollisionAlgorithm__processCollision(btCollisionAlgorithm* self, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo* dispatchInfo, btManifoldResult* resultOut)
{
	self->processCollision(body0Wrap, body1Wrap, *dispatchInfo, resultOut);
}

CLASSFUNC(btCollisionAlgorithm)
btScalar DLLAPI btCollisionAlgorithm__calculateTimeOfImpact(btCollisionAlgorithm* self, btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo* dispatchInfo, btManifoldResult* resultOut)
{
	self->calculateTimeOfImpact(body0, body1, *dispatchInfo, resultOut);
}

CLASSFUNC(btCollisionAlgorithm)
void DLLAPI btCollisionAlgorithm__getAllContactManifolds(btCollisionAlgorithm* self, btManifoldArray* manifoldArray)
{
	self->getAllContactManifolds(*manifoldArray);
}
#endif

//=============================================================================
// btTypedConstraint
//=============================================================================

CLASSFUNC(btTypedConstraint)
DESTRUCTOR void DLLAPI btTypedConstraint__destroy(btTypedConstraint* self)
{
	delete self;
}


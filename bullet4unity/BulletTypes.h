#ifndef _BULLETTYPES_H_
#define _BULLETTYPES_H_

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBox2dShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCollisionMargin.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btConvex2dShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btConvexInternalShape.h>
#include <BulletCollision/CollisionShapes/btConvexPointCloudShape.h>
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btMaterial.h>
#include <BulletCollision/CollisionShapes/btMiniSDF.h>
#include <BulletCollision/CollisionShapes/btMinkowskiSumShape.h>
#include <BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btMultiSphereShape.h>
#include <BulletCollision/CollisionShapes/btOptimizedBvh.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btStridingMeshInterface.h>
#include <BulletCollision/CollisionShapes/btTetrahedronShape.h>
#include <BulletCollision/CollisionShapes/btTriangleBuffer.h>
#include <BulletCollision/CollisionShapes/btTriangleCallback.h>
#include <BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h>
#include <BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h>
#include <BulletCollision/CollisionShapes/btTriangleInfoMap.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/CollisionShapes/btUniformScalingShape.h>

#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/BroadphaseCollision/btSimpleBroadphase.h>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>

#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btUniversalConstraint.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSolverConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHinge2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGearConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btContactConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btActionInterface.h>

#include <BulletDynamics/Character/btKinematicCharacterController.h>

#include <LinearMath/btPoolAllocator.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btIDebugDraw.h>

enum ActivationStates
{
	Nnone = 0,
	Active = ACTIVE_TAG,
	IslandSleeping = ISLAND_SLEEPING,
	WantsDeactivation = WANTS_DEACTIVATION,
	DisableDeactivation = DISABLE_DEACTIVATION,
	DisableSimulation = DISABLE_SIMULATION,
};

enum DebugDrawFlags
{
	NoDebug = btIDebugDraw::DBG_NoDebug,
	DrawWireframe = btIDebugDraw::DBG_DrawWireframe,
	DrawAabb = btIDebugDraw::DBG_DrawAabb,
	DrawFeaturesText = btIDebugDraw::DBG_DrawFeaturesText,
	DrawContactPoints = btIDebugDraw::DBG_DrawContactPoints,
	NoDeactivation = btIDebugDraw::DBG_NoDeactivation,
	NoHelpText = btIDebugDraw::DBG_NoHelpText,
	DrawText = btIDebugDraw::DBG_DrawText,
	ProfileTimings = btIDebugDraw::DBG_ProfileTimings,
	EnableSatComparison = btIDebugDraw::DBG_EnableSatComparison,
	DisableBulletLCP = btIDebugDraw::DBG_DisableBulletLCP,
	EnableCCD = btIDebugDraw::DBG_EnableCCD,
	DrawConstraints = btIDebugDraw::DBG_DrawConstraints,
	DrawConstraintLimits = btIDebugDraw::DBG_DrawConstraintLimits,
	FastWireframe = btIDebugDraw::DBG_FastWireframe,
	DrawNormals = btIDebugDraw::DBG_DrawNormals,
	DrawFrames = btIDebugDraw::DBG_DrawFrames,
};

typedef unsigned int uint;

#endif // _BULLETTYPES_H_

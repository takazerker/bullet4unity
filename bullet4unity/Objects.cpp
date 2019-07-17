#include "BulletTypes.h"
#include "UnityTypes.h"

//=============================================================================
// btCollisionObject
//=============================================================================

CLASSFUNC(btCollisionObject)
STATIC btCollisionObject* DLLAPI btCollisionObject__create_0()
{
	return new btCollisionObject();
}

CLASSFUNC(btCollisionObject)
DESTRUCTOR void DLLAPI btCollisionObject__destroy(btCollisionObject* self)
{
	delete self;
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__mergesSimulationIslands_0(btCollisionObject* self)
{
	return self->mergesSimulationIslands();
}

CLASSFUNC(btCollisionObject)
Vector3 DLLAPI btCollisionObject__getAnisotropicFriction_0(btCollisionObject* self)
{
	return self->getAnisotropicFriction();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setAnisotropicFriction_0(btCollisionObject* self, const Vector3* anisotropicFriction, btCollisionObject::AnisotropicFrictionFlags frictionMode)
{
	self->setAnisotropicFriction(anisotropicFriction->Marshall(), frictionMode);
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__hasAnisotropicFriction_0(btCollisionObject* self, btCollisionObject::AnisotropicFrictionFlags frictionMode)
{
	return self->hasAnisotropicFriction(frictionMode);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setContactProcessingThreshold_0(btCollisionObject* self, float contactProcessingThreshold)
{
	self->setContactProcessingThreshold(contactProcessingThreshold);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getContactProcessingThreshold_0(btCollisionObject* self)
{
	return self->getContactProcessingThreshold();
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__isStaticObject_0(btCollisionObject* self)
{
	return self->isStaticObject();
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__isKinematicObject_0(btCollisionObject* self)
{
	return self->isKinematicObject();
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__isStaticOrKinematicObject_0(btCollisionObject* self)
{
	return self->isStaticOrKinematicObject();
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__hasContactResponse_0(btCollisionObject* self)
{
	return self->hasContactResponse();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setCollisionShape_0(btCollisionObject* self, btCollisionShape* collisionShape)
{
	self->setCollisionShape(collisionShape);
}

CLASSFUNC(btCollisionObject)
btCollisionShape* DLLAPI btCollisionObject__getCollisionShape_0(btCollisionObject* self)
{
	return self->getCollisionShape();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setIgnoreCollisionCheck_0(btCollisionObject* self, btCollisionObject* co, bool ignoreCollisionCheck)
{
	self->setIgnoreCollisionCheck(co, ignoreCollisionCheck);
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__checkCollideWithOverride_0(btCollisionObject*self, btCollisionObject* co)
{
return self->checkCollideWithOverride(co);
}

CLASSFUNC(btCollisionObject)
ActivationStates DLLAPI btCollisionObject__getActivationState_0(btCollisionObject* self)
{
	return (ActivationStates)self->getActivationState();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setActivationState_0(btCollisionObject* self, ActivationStates newState)
{
	self->setActivationState(newState);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setDeactivationTime_0(btCollisionObject* self, float time)
{
	self->setDeactivationTime(time);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getDeactivationTime_0(btCollisionObject* self)
{
	return self->getDeactivationTime();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__forceActivationState_0(btCollisionObject* self, int newState)
{
	self->forceActivationState(newState);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__activate_0(btCollisionObject* self, bool forceActivation = false)
{
	self->activate(forceActivation);
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__isActive_0(btCollisionObject* self)
{
	return self->isActive();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setRestitution_0(btCollisionObject* self, float rest)
{
	self->setRestitution(rest);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getRestitution_0(btCollisionObject* self)
{
	return self->getRestitution();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setFriction_0(btCollisionObject* self, float frict)
{
	self->setFriction(frict);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getFriction_0(btCollisionObject* self)
{
	return self->getFriction();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setRollingFriction_0(btCollisionObject* self, float frict)
{
	self->setRollingFriction(frict);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getRollingFriction_0(btCollisionObject* self)
{
	return self->getRollingFriction();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setSpinningFriction_0(btCollisionObject* self, float frict)
{
	self->setSpinningFriction(frict);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getSpinningFriction_0(btCollisionObject* self)
{
	return self->getSpinningFriction();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setContactStiffnessAndDamping_0(btCollisionObject* self, float stiffness, float damping)
{
	self->setContactStiffnessAndDamping(stiffness, damping);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getContactStiffness_0(btCollisionObject* self)
{
	return self->getContactStiffness();
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getContactDamping_0(btCollisionObject* self)
{
	return self->getContactDamping();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__getWorldTransform_0(btCollisionObject* self, btTransform* worldTrans)
{
	*AlignAddressOf(worldTrans) = self->getWorldTransform();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setWorldTransform_0(btCollisionObject* self, const btTransform* worldTrans)
{
	self->setWorldTransform(*AlignAddressOf(worldTrans));
}

CLASSFUNC(btCollisionObject)
btBroadphaseProxy* DLLAPI btCollisionObject__getBroadphaseHandle_0(btCollisionObject* self)
{
	return self->getBroadphaseHandle();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setBroadphaseHandle_0(btCollisionObject* self, btBroadphaseProxy* handle)
{
	self->setBroadphaseHandle(handle);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__getInterpolationWorldTransform_0(btCollisionObject* self, btTransform* trans)
{
	*AlignAddressOf(trans) = self->getInterpolationWorldTransform();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setInterpolationWorldTransform_0(btCollisionObject* self, const btTransform* trans)
{
	self->setInterpolationWorldTransform(*AlignAddressOf(trans));
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setInterpolationLinearVelocity_0(btCollisionObject* self, const Vector3* linvel)
{
	self->setInterpolationLinearVelocity(linvel->Marshall());
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setInterpolationAngularVelocity_0(btCollisionObject* self, const Vector3* angvel)
{
	self->setInterpolationAngularVelocity(angvel->Marshall());
}

CLASSFUNC(btCollisionObject)
Vector3 DLLAPI btCollisionObject__getInterpolationLinearVelocity_0(btCollisionObject* self)
{
	return self->getInterpolationLinearVelocity();
}

CLASSFUNC(btCollisionObject)
Vector3 DLLAPI btCollisionObject__getInterpolationAngularVelocity_0(btCollisionObject* self)
{
	return self->getInterpolationAngularVelocity();
}

CLASSFUNC(btCollisionObject)
int DLLAPI btCollisionObject__getIslandTag_0(btCollisionObject* self)
{
	return self->getIslandTag();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setIslandTag_0(btCollisionObject* self, int tag)
{
	self->setIslandTag(tag);
}

CLASSFUNC(btCollisionObject)
int DLLAPI btCollisionObject__getCompanionId_0(btCollisionObject* self)
{
	return self->getCompanionId();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setCompanionId_0(btCollisionObject* self, int id)
{
	self->setCompanionId(id);
}

CLASSFUNC(btCollisionObject)
int DLLAPI btCollisionObject__getWorldArrayIndex_0(btCollisionObject* self)
{
	return self->getWorldArrayIndex();
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getHitFraction_0(btCollisionObject* self)
{
	return self->getHitFraction();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setHitFraction_0(btCollisionObject* self, float hitFraction)
{
	self->setHitFraction(hitFraction);
}

CLASSFUNC(btCollisionObject)
btCollisionObject::CollisionFlags DLLAPI btCollisionObject__getCollisionFlags_0(btCollisionObject* self)
{
	return (btCollisionObject::CollisionFlags)self->getCollisionFlags();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setCollisionFlags_0(btCollisionObject* self, btCollisionObject::CollisionFlags flags)
{
	self->setCollisionFlags(flags);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getCcdSweptSphereRadius_0(btCollisionObject* self)
{
	return self->getCcdSweptSphereRadius();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setCcdSweptSphereRadius_0(btCollisionObject* self, float radius)
{
	self->setCcdSweptSphereRadius(radius);
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getCcdMotionThreshold_0(btCollisionObject* self)
{
	return self->getCcdMotionThreshold();
}

CLASSFUNC(btCollisionObject)
float DLLAPI btCollisionObject__getCcdSquareMotionThreshold_0(btCollisionObject* self)
{
	return self->getCcdSquareMotionThreshold();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setCcdMotionThreshold_0(btCollisionObject* self, float ccdMotionThreshold)
{
	self->setCcdMotionThreshold(ccdMotionThreshold);
}

CLASSFUNC(btCollisionObject)
void* DLLAPI btCollisionObject__getUserPointer_0(btCollisionObject* self)
{
	return self->getUserPointer();
}

CLASSFUNC(btCollisionObject)
int DLLAPI btCollisionObject__getUserIndex_0(btCollisionObject* self)
{
	return self->getUserIndex();
}

CLASSFUNC(btCollisionObject)
int DLLAPI btCollisionObject__getUserIndex2_0(btCollisionObject* self)
{
	return self->getUserIndex2();
}

CLASSFUNC(btCollisionObject)
int DLLAPI btCollisionObject__getUserIndex3_0(btCollisionObject* self)
{
	return self->getUserIndex3();
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setUserPointer(btCollisionObject* self, void* userPointer)
{
	self->setUserPointer(userPointer);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setUserIndex_0(btCollisionObject* self, int index)
{
	self->setUserIndex(index);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setUserIndex2_0(btCollisionObject* self, int index)
{
	self->setUserIndex2(index);
}

CLASSFUNC(btCollisionObject)
void DLLAPI btCollisionObject__setUserIndex3_0(btCollisionObject* self, int index)
{
	self->setUserIndex3(index);
}

CLASSFUNC(btCollisionObject)
bool DLLAPI btCollisionObject__checkCollideWith_0(btCollisionObject* self, btCollisionObject* co)
{
	return self->checkCollideWith(co);
}

//=============================================================================
// btRigidBody
//=============================================================================

CLASSFUNC(btRigidBody)
STATIC btRigidBody* DLLAPI btRigidBody__create_0(float mass, btMotionState* motionState, btCollisionShape* collisionShape, const Vector3* localInertia)
{
	return new btRigidBody(mass, motionState, collisionShape, localInertia->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__proceedToTransform_0(btRigidBody* self, const btTransform* newTrans)
{
	self->proceedToTransform(*AlignAddressOf(newTrans));
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__predictIntegratedTransform_0(btRigidBody* self, float step, btTransform* predictedTransform)
{
	self->predictIntegratedTransform(step, *AlignAddressOf(predictedTransform));
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__saveKinematicState_0(btRigidBody* self, float step)
{
	self->saveKinematicState(step);
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyGravity_0(btRigidBody* self)
{
	self->applyGravity();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setGravity_0(btRigidBody* self, const Vector3* acceleration)
{
	self->setGravity(acceleration->Marshall());
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getGravity_0(btRigidBody* self)
{
	return self->getGravity();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setDamping_0(btRigidBody* self, float lin_damping, float ang_damping)
{
	self->setDamping(lin_damping, ang_damping);
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__getLinearDamping_0(btRigidBody* self)
{
	return self->getLinearDamping();
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__getAngularDamping_0(btRigidBody* self)
{
	return self->getAngularDamping();
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__getLinearSleepingThreshold_0(btRigidBody* self)
{
	return self->getLinearSleepingThreshold();
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__getAngularSleepingThreshold_0(btRigidBody* self)
{
	return self->getAngularSleepingThreshold();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyDamping_0(btRigidBody* self, float timeStep)
{
	self->applyDamping(timeStep);
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setMassProps_0(btRigidBody* self, float mass, const Vector3* inertia)
{
	self->setMassProps(mass, inertia->Marshall());
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getLinearFactor_0(btRigidBody* self)
{
	return self->getLinearFactor();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setLinearFactor_0(btRigidBody* self, const Vector3* linearFactor)
{
	self->setLinearFactor(linearFactor->Marshall());
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__getInvMass_0(btRigidBody* self)
{
	return self->getInvMass();
}

CLASSFUNC(btRigidBody)
Matrix4x4 DLLAPI btRigidBody__getInvInertiaTensorWorld_0(btRigidBody* self)
{
	return self->getInvInertiaTensorWorld();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__integrateVelocities_0(btRigidBody* self, float step)
{
	self->integrateVelocities(step);
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setCenterOfMassTransform_0(btRigidBody* self, btTransform* xform)
{
	self->setCenterOfMassTransform(*AlignAddressOf(xform));
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyCentralForce_0(btRigidBody* self, const Vector3* force)
{
	self->applyCentralForce(force->Marshall());
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getTotalForce_0(btRigidBody* self)
{
	return self->getTotalForce();
};

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getTotalTorque_0(btRigidBody* self)
{
	return self->getTotalTorque();
};

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getInvInertiaDiagLocal_0(btRigidBody* self)
{
	return self->getInvInertiaDiagLocal();
};

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setInvInertiaDiagLocal_0(btRigidBody* self, const Vector3* diagInvInertia)
{
	self->setInvInertiaDiagLocal(diagInvInertia->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setSleepingThresholds_0(btRigidBody* self, float linear, float angular)
{
	self->setSleepingThresholds(linear, angular);
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyTorque_0(btRigidBody* self, const Vector3* torque)
{
	self->applyTorque(torque->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyForce_0(btRigidBody* self, const Vector3* force, const Vector3* rel_pos)
{
	self->applyForce(force->Marshall(), rel_pos->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyCentralImpulse_0(btRigidBody* self, const Vector3* impulse)
{
	self->applyCentralImpulse(impulse->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyTorqueImpulse_0(btRigidBody* self, const Vector3* torque)
{
	self->applyTorqueImpulse(torque->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__applyImpulse_0(btRigidBody* self, const Vector3* impulse, const Vector3* rel_pos)
{
	self->applyImpulse(impulse->Marshall(), rel_pos->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__clearForces_0(btRigidBody* self)
{
	self->clearForces();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__updateInertiaTensor_0(btRigidBody* self)
{
	self->updateInertiaTensor();
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getCenterOfMassPosition_0(btRigidBody* self)
{
	return self->getCenterOfMassPosition();
}

CLASSFUNC(btRigidBody)
Quaternion DLLAPI btRigidBody__getOrientation_0(btRigidBody* self)
{
	return self->getOrientation();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__getCenterOfMassTransform_0(btRigidBody* self, btTransform* trans)
{
	*AlignAddressOf(trans) = self->getCenterOfMassTransform();
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getLinearVelocity_0(btRigidBody* self)
{
	return self->getLinearVelocity();
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getAngularVelocity_0(btRigidBody* self)
{
	return self->getAngularVelocity();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setLinearVelocity_0(btRigidBody* self, const Vector3* lin_vel)
{
	self->setLinearVelocity(lin_vel->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setAngularVelocity_0(btRigidBody* self, const Vector3* ang_vel)
{
	self->setAngularVelocity(ang_vel->Marshall());
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getVelocityInLocalPoint_0(btRigidBody* self, Vector3* rel_pos)
{
	return self->getVelocityInLocalPoint(rel_pos->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__translate_0(btRigidBody* self, const Vector3* v)
{
	self->translate(v->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__getAabb_0(btRigidBody* self, Vector3* aabbMin, Vector3* aabbMax)
{
	self->getAabb(aabbMin->Marshall(), aabbMax->Marshall());
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__computeImpulseDenominator_0(btRigidBody* self, const Vector3* pos, const Vector3* normal)
{
	return self->computeImpulseDenominator(pos->Marshall(), normal->Marshall());
}

CLASSFUNC(btRigidBody)
float DLLAPI btRigidBody__computeAngularImpulseDenominator_0(btRigidBody* self, const Vector3* axis)
{
	return self->computeAngularImpulseDenominator(axis->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__updateDeactivation_0(btRigidBody* self, float timeStep)
{
	self->updateDeactivation(timeStep);
}

CLASSFUNC(btRigidBody)
bool DLLAPI btRigidBody__wantsSleeping_0(btRigidBody* self)
{
	return self->wantsSleeping();
}

CLASSFUNC(btRigidBody)
btBroadphaseProxy* DLLAPI btRigidBody__getBroadphaseProxy_0(btRigidBody* self)
{
	return self->getBroadphaseProxy();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setNewBroadphaseProxy_0(btRigidBody* self, btBroadphaseProxy* broadphaseProxy)
{
	self->setNewBroadphaseProxy(broadphaseProxy);
}

CLASSFUNC(btRigidBody)
btMotionState* DLLAPI btRigidBody__getMotionState_0(btRigidBody* self)
{
	return self->getMotionState();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setMotionState_0(btRigidBody* self, btMotionState* motionState)
{
	self->setMotionState(motionState);
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setAngularFactorVector_0(btRigidBody* self, const Vector3* angFac)
{
	self->setAngularFactor(angFac->Marshall());
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setAngularFactorFloat_0(btRigidBody* self, float angFac)
{
	self->setAngularFactor(angFac);
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getAngularFactor_0(btRigidBody* self)
{
	return self->getAngularFactor();
}

CLASSFUNC(btRigidBody)
bool DLLAPI btRigidBody__isInWorld_0(btRigidBody* self)
{
	return self->isInWorld();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__addConstraintRef_0(btRigidBody* self, btTypedConstraint* c)
{
	self->addConstraintRef(c);
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__removeConstraintRef_0(btRigidBody* self, btTypedConstraint* c)
{
	self->removeConstraintRef(c);
}

CLASSFUNC(btRigidBody)
btTypedConstraint* DLLAPI btRigidBody__getConstraintRef_0(btRigidBody* self, int index)
{
	return self->getConstraintRef(index);
}

CLASSFUNC(btRigidBody)
int DLLAPI btRigidBody__getNumConstraintRefs_0(btRigidBody* self)
{
	return self->getNumConstraintRefs();
}

CLASSFUNC(btRigidBody)
void DLLAPI btRigidBody__setFlags_0(btRigidBody* self, int flags)
{
	self->setFlags(flags);
}

CLASSFUNC(btRigidBody)
int DLLAPI btRigidBody__getFlags_0(btRigidBody* self)
{
	return self->getFlags();
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__computeGyroscopicImpulseImplicit_World_0(btRigidBody* self, float dt)
{
	return self->computeGyroscopicImpulseImplicit_World(dt);
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__computeGyroscopicImpulseImplicit_Body_0(btRigidBody* self, float step)
{
	return self->computeGyroscopicImpulseImplicit_Body(step);
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__computeGyroscopicForceExplicit_0(btRigidBody* self, float maxGyroscopicForce)
{
	return self->computeGyroscopicForceExplicit(maxGyroscopicForce);
}

CLASSFUNC(btRigidBody)
Vector3 DLLAPI btRigidBody__getLocalInertia_0(btRigidBody* self)
{
	return self->getLocalInertia();
}

//=============================================================================
// btDefaultMotionState
//=============================================================================

CLASSFUNC(btMotionState)
DESTRUCTOR void DLLAPI btMotionState__destroy(btMotionState* self)
{
	delete self;
}

//=============================================================================
// btDefaultMotionState
//=============================================================================

CLASSFUNC(btDefaultMotionState)
STATIC btDefaultMotionState* DLLAPI btDefaultMotionState__create_0()
{
	return new btDefaultMotionState();
}

CLASSFUNC(btDefaultMotionState)
void DLLAPI btDefaultMotionState__getWorldTransform_0(btDefaultMotionState* self, btTransform* centerOfMassWorldTrans)
{
	self->getWorldTransform(*AlignAddressOf(centerOfMassWorldTrans));
}

CLASSFUNC(btDefaultMotionState)
void DLLAPI btDefaultMotionState__setWorldTransform_0(btDefaultMotionState* self, const btTransform* centerOfMassWorldTrans)
{
	self->setWorldTransform(*AlignAddressOf(centerOfMassWorldTrans));
}

//=============================================================================
// btGhostPairCallback
//=============================================================================

CLASSFUNC(btOverlappingPairCallback)
DESTRUCTOR void DLLAPI btOverlappingPairCallback__destroy(btOverlappingPairCallback* self)
{
	delete self;
}

//=============================================================================
// btGhostPairCallback
//=============================================================================

CLASSFUNC(btGhostPairCallback)
STATIC btGhostPairCallback* DLLAPI btGhostPairCallback__create_0()
{
	return new btGhostPairCallback();
}

//=============================================================================
// btGhostObject
//=============================================================================

CLASSFUNC(btGhostObject)
STATIC btGhostObject* DLLAPI btGhostObject__create_0()
{
	return new btGhostObject();
}

CLASSFUNC(btGhostObject)
void DLLAPI btGhostObject__convexSweepTest_0(btGhostObject* self, btConvexShape* castShape, const btTransform* convexFromWorld, btTransform* convexToWorld, btCollisionWorld::ConvexResultCallback* resultCallback, float allowedCcdPenetration = 0.0f)
{
	self->convexSweepTest(castShape, *AlignAddressOf(convexFromWorld), *AlignAddressOf(convexToWorld), *resultCallback, allowedCcdPenetration);
}

CLASSFUNC(btGhostObject)
void DLLAPI btGhostObject__rayTest_0(btGhostObject* self, const Vector3* rayFromWorld, const Vector3* rayToWorld, btCollisionWorld::RayResultCallback* resultCallback)
{
	self->rayTest(rayFromWorld->Marshall(), rayToWorld->Marshall(), *resultCallback);
}

CLASSFUNC(btGhostObject)
int DLLAPI btGhostObject__getNumOverlappingObjects_0(btGhostObject* self)
{
	return self->getNumOverlappingObjects();
}

CLASSFUNC(btGhostObject)
btCollisionObject* DLLAPI btGhostObject__getOverlappingObject(btGhostObject* self, int index)
{
	return self->getOverlappingObject(index);
}

//=============================================================================
// btPairCachingGhostObject
//=============================================================================

CLASSFUNC(btPairCachingGhostObject)
STATIC btPairCachingGhostObject* DLLAPI btPairCachingGhostObject__create_0()
{
	return new btPairCachingGhostObject();
}

CLASSFUNC(btPairCachingGhostObject)
btHashedOverlappingPairCache* DLLAPI btPairCachingGhostObject__getOverlappingPairCache_0(btPairCachingGhostObject* self)
{
	return self->getOverlappingPairCache();
}

//=============================================================================
// btActionInterface
//=============================================================================

CLASSFUNC(btActionInterface)
DESTRUCTOR void DLLAPI btActionInterface__destroy(btActionInterface* self)
{
	delete self;
}

//=============================================================================
// btKinematicCharacterController
//=============================================================================

CLASSFUNC(btKinematicCharacterController)
STATIC btKinematicCharacterController* DLLAPI btKinematicCharacterController__create_0(btPairCachingGhostObject* ghostObject, btConvexShape* convexShape, float stepHeight, const Vector3* up)
{
	return new btKinematicCharacterController(ghostObject, convexShape, stepHeight, up->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setUp_0(btKinematicCharacterController* self, const Vector3* up)
{
	self->setUp(up->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
Vector3 DLLAPI btKinematicCharacterController__getUp_0(btKinematicCharacterController* self)
{
	return self->getUp();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setWalkDirection_0(btKinematicCharacterController* self, const Vector3* walkDirection)
{
	self->setWalkDirection(walkDirection->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setVelocityForTimeInterval_0(btKinematicCharacterController* self, const Vector3* velocity, float timeInterval)
{
	self->setVelocityForTimeInterval(velocity->Marshall(), timeInterval);
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setAngularVelocity_0(btKinematicCharacterController* self, const Vector3* velocity)
{
	self->setAngularVelocity(velocity->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
Vector3 DLLAPI btKinematicCharacterController__getAngularVelocity_0(btKinematicCharacterController* self)
{
	return self->getAngularVelocity();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setLinearVelocity_0(btKinematicCharacterController* self, const Vector3* velocity)
{
	self->setLinearVelocity(velocity->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
Vector3 DLLAPI btKinematicCharacterController__getLinearVelocity_0(btKinematicCharacterController* self)
{
	return self->getLinearVelocity();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setLinearDamping_0(btKinematicCharacterController* self, float d)
{
	self->setLinearDamping(d);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getLinearDamping_0(btKinematicCharacterController* self)
{
	return self->getLinearDamping();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setAngularDamping_0(btKinematicCharacterController* self, float d)
{
	self->setAngularDamping(d);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getAngularDamping_0(btKinematicCharacterController* self)
{
	return self->getAngularDamping();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__reset_0(btKinematicCharacterController* self, btCollisionWorld* collisionWorld)
{
	self->reset(collisionWorld);
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__warp_0(btKinematicCharacterController* self, const Vector3* origin)
{
	self->warp(origin->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__preStep_0(btKinematicCharacterController* self, btCollisionWorld * collisionWorld)
{
	self->preStep(collisionWorld);
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__playerStep_0(btKinematicCharacterController* self, btCollisionWorld * collisionWorld, float dt)
{
	self->playerStep(collisionWorld, dt);
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setStepHeight_0(btKinematicCharacterController* self, float h)
{
	self->setStepHeight(h);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getStepHeight_0(btKinematicCharacterController* self)
{
	return self->getStepHeight();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setJumpSpeed_0(btKinematicCharacterController* self, float jumpSpeed)
{
	self->setJumpSpeed(jumpSpeed);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getJumpSpeed_0(btKinematicCharacterController* self)
{
	return self->getJumpSpeed();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setFallSpeed_0(btKinematicCharacterController* self, float fallSpeed)
{
	self->setFallSpeed(fallSpeed);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getFallSpeed_0(btKinematicCharacterController* self)
{
	return self->getFallSpeed();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setMaxJumpHeight_0(btKinematicCharacterController* self, float maxJumpHeight)
{
	self->setMaxJumpHeight(maxJumpHeight);
}

CLASSFUNC(btKinematicCharacterController)
bool DLLAPI btKinematicCharacterController__canJump_0(btKinematicCharacterController* self)
{
	return self->canJump();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__jump_0(btKinematicCharacterController* self, const Vector3* v)
{
	self->jump(v->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__applyImpulse_0(btKinematicCharacterController* self, const Vector3* v)
{
	self->applyImpulse(v->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setGravity_0(btKinematicCharacterController* self, const Vector3* gravity)
{
	self->setGravity(gravity->Marshall());
}

CLASSFUNC(btKinematicCharacterController)
Vector3 DLLAPI btKinematicCharacterController__getGravity_0(btKinematicCharacterController* self)
{
	return self->getGravity();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setMaxSlope_0(btKinematicCharacterController* self, float slopeRadians)
{
	self->setMaxSlope(slopeRadians);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getMaxSlope_0(btKinematicCharacterController* self)
{
	return self->getMaxSlope();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setMaxPenetrationDepth_0(btKinematicCharacterController* self, float d)
{
	self->setMaxPenetrationDepth(d);
}

CLASSFUNC(btKinematicCharacterController)
float DLLAPI btKinematicCharacterController__getMaxPenetrationDepth_0(btKinematicCharacterController* self)
{
	return self->getMaxPenetrationDepth();
}

CLASSFUNC(btKinematicCharacterController)
btPairCachingGhostObject* DLLAPI btKinematicCharacterController__getGhostObject_0(btKinematicCharacterController* self)
{
	return self->getGhostObject();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setUseGhostSweepTest_0(btKinematicCharacterController* self, bool useGhostObjectSweepTest)
{
	self->setUseGhostSweepTest(useGhostObjectSweepTest);
}

CLASSFUNC(btKinematicCharacterController)
bool DLLAPI btKinematicCharacterController__onGround_0(btKinematicCharacterController* self)
{
	return self->onGround();
}

CLASSFUNC(btKinematicCharacterController)
void DLLAPI btKinematicCharacterController__setUpInterpolate(btKinematicCharacterController* self, bool value)
{
	self->setUpInterpolate(value);
}


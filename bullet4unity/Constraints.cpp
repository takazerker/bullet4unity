#include "BulletTypes.h"
#include "UnityTypes.h"

//=============================================================================
// btConeTwistConstraint
//=============================================================================

CLASSFUNC(btConeTwistConstraint)
STATIC btConeTwistConstraint* DLLAPI btConeTwistConstraint__create_0(btRigidBody* rbA, btRigidBody* rbB, const btTransform* rbAFrame, const btTransform* rbBFrame)
{
	return new btConeTwistConstraint(*rbA, *rbB, *AlignAddressOf(rbAFrame), *AlignAddressOf(rbBFrame));
}

CLASSFUNC(btConeTwistConstraint)
STATIC btConeTwistConstraint* DLLAPI btConeTwistConstraint__create_1(btRigidBody* rbA, const btTransform* rbAFrame)
{
	return new btConeTwistConstraint(*rbA, *AlignAddressOf(rbAFrame));
}

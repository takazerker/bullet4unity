#include "BulletTypes.h"
#include "UnityTypes.h"

EXPORT_ALIGNED(btTransform)

//=============================================================================
// btTransform
//=============================================================================

CLASSFUNC(btTransform)
Vector3 DLLAPI btTransform__getOrigin_0(btTransform* self)
{
	return AlignAddressOf(self)->getOrigin();
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__setOrigin_0(btTransform* self, const Vector3* origin)
{
	AlignAddressOf(self)->setOrigin(origin->Marshall());
}

CLASSFUNC(btTransform)
Quaternion DLLAPI btTransform__getRotation_0(btTransform* self)
{
	return AlignAddressOf(self)->getRotation();
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__setRotation_0(btTransform* self, const Quaternion* rotation)
{
	AlignAddressOf(self)->setRotation(rotation->Marshall());
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__setIdentity_0(btTransform* self)
{
	AlignAddressOf(self)->setIdentity();
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__mult_0(btTransform* self, const btTransform* t1, const btTransform* t2)
{
	AlignAddressOf(self)->mult(*AlignAddressOf(t1), *AlignAddressOf(t2));
}

CLASSFUNC(btTransform)
Vector3 DLLAPI btTransform__mult_1(btTransform* self, const Vector3* v)
{
	return *AlignAddressOf(self) * v->Marshall();
}

CLASSFUNC(btTransform)
Quaternion DLLAPI btTransform__mult_2(btTransform* self, const Quaternion* q)
{
	return *AlignAddressOf(self) * q->Marshall();
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__mult_3(btTransform* self, const btTransform* t1)
{
	*AlignAddressOf(self) *= *AlignAddressOf(t1);
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__inverse_0(btTransform* self, btTransform* result)
{
	*AlignAddressOf(result) = AlignAddressOf(self)->inverse();
}

CLASSFUNC(btTransform)
void DLLAPI btTransform__inverseTimes_0(btTransform* self, btTransform* result, const btTransform* t)
{
	*AlignAddressOf(result) = AlignAddressOf(self)->inverseTimes(*AlignAddressOf(t));
}


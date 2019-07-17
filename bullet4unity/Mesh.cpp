#include "BulletTypes.h"
#include "UnityTypes.h"
#include "btManagedMesh.h"

//=============================================================================
// btManagedMesh
//=============================================================================

CLASSFUNC(btManagedMesh)
STATIC btManagedMesh* DLLAPI btManagedMesh__create_0(int instanceID, int numTriangles, void* triangleIndexBase, int triangleIndexStride, int numVertices, void* vertexBase, int vertexStride)
{
	auto result = new btManagedMesh(instanceID, numTriangles, (int*)triangleIndexBase, triangleIndexStride, numVertices, (btScalar*)vertexBase, vertexStride);
	result->addRefCount();
	return result;
}

CLASSFUNC(btManagedMesh)
STATIC btManagedMesh* DLLAPI btManagedMesh__get_0(int instanceID)
{
	auto itr = _managedMeshes.find(instanceID);
	if (itr != _managedMeshes.end())
	{
		itr->second->addRefCount();
		return itr->second;
	}
	return 0;
}

CLASSFUNC(btManagedMesh)
int DLLAPI btManagedMesh__addRefCount_0(btManagedMesh* self)
{
	return self->addRefCount();
}

CLASSFUNC(btManagedMesh)
int DLLAPI btManagedMesh__release_0(btManagedMesh* self)
{
	return self->release();
}

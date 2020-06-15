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

//=============================================================================
// btSingleMesh
//=============================================================================

CLASSFUNC(btSingleMesh)
STATIC btSingleMesh* DLLAPI btSingleMesh__create_0(PHY_ScalarType indexType, int numTriangles, void* triangleIndexBase, int triangleIndexStride, PHY_ScalarType vertexType, int numVertices, void* vertexBase, int vertexStride, const Vector3* aabbMin, const Vector3* aabbMax)
{
	return new btSingleMesh(indexType, numTriangles, triangleIndexBase, triangleIndexStride, vertexType, numVertices, vertexBase, vertexStride, aabbMin, aabbMax);
}

CLASSFUNC(btSingleMesh)
void DLLAPI btSingleMesh__destroy_0(btSingleMesh* self)
{
	delete self;
}

btSingleMesh::btSingleMesh(PHY_ScalarType indexType, int numTriangles, void* triangleIndexBase, int triangleIndexStride, PHY_ScalarType vertexType, int numVertices, void* vertexBase, int vertexStride, const Vector3* aabbMin, const Vector3* aabbMax)
	: m_aabbMin(aabbMin->Marshall())
	, m_aabbMax(aabbMax->Marshall())
{
	m_mesh.m_indexType = indexType;
	m_mesh.m_vertexType = vertexType;
	m_mesh.m_numTriangles = numTriangles;
	m_mesh.m_triangleIndexBase = (unsigned char*)triangleIndexBase;
	m_mesh.m_triangleIndexStride = triangleIndexStride * 3;
	m_mesh.m_numVertices = numVertices;
	m_mesh.m_vertexBase = (unsigned char*)vertexBase;
	m_mesh.m_vertexStride = vertexStride;
}

btSingleMesh::~btSingleMesh()
{
}

void btSingleMesh::getLockedVertexIndexBase(unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart)
{
	btAssert(subpart < getNumSubParts());

	btIndexedMesh& mesh = m_mesh;

	numverts = mesh.m_numVertices;
	(*vertexbase) = (unsigned char*)mesh.m_vertexBase;

	type = mesh.m_vertexType;

	vertexStride = mesh.m_vertexStride;

	numfaces = mesh.m_numTriangles;

	(*indexbase) = (unsigned char*)mesh.m_triangleIndexBase;
	indexstride = mesh.m_triangleIndexStride;
	indicestype = mesh.m_indexType;
}

void btSingleMesh::getLockedReadOnlyVertexIndexBase(const unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, const unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart) const
{
	const btIndexedMesh& mesh = m_mesh;

	numverts = mesh.m_numVertices;
	(*vertexbase) = (const unsigned char*)mesh.m_vertexBase;

	type = mesh.m_vertexType;

	vertexStride = mesh.m_vertexStride;

	numfaces = mesh.m_numTriangles;
	(*indexbase) = (const unsigned char*)mesh.m_triangleIndexBase;
	indexstride = mesh.m_triangleIndexStride;
	indicestype = mesh.m_indexType;
}

bool btSingleMesh::hasPremadeAabb() const
{
	return true;
}

void btSingleMesh::setPremadeAabb(const btVector3& aabbMin, const btVector3& aabbMax) const
{
}

void btSingleMesh::getPremadeAabb(btVector3* aabbMin, btVector3* aabbMax) const
{
	*aabbMin = m_aabbMin;
	*aabbMax = m_aabbMax;
}

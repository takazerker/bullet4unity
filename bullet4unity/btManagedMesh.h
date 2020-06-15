#ifndef _BTMANAGEDMESH_H_
#define _BTMANAGEDMESH_H_
#include "BulletTypes.h"
#include <map>

static std::map<int, class btManagedMesh*> _managedMeshes;

class btManagedMesh : public btTriangleIndexVertexArray
{
public:

	int m_instanceID;
	int m_refCount;

	static void* copyBuffer(void* src, int stride, int count)
	{
		char* buf = new char[stride * count];
		memcpy(buf, src, stride * count);
		return buf;
	}

	btManagedMesh(int instanceID, int numTriangles, int* triangleIndexBase, int triangleIndexStride, int numVertices, btScalar* vertexBase, int vertexStride)
		: m_refCount(0)
		, m_instanceID(instanceID)
	{
		btIndexedMesh mesh;
		mesh.m_numTriangles = numTriangles;
		mesh.m_triangleIndexBase = (const unsigned char*)copyBuffer(triangleIndexBase, triangleIndexStride, numTriangles * 3);
		mesh.m_triangleIndexStride = triangleIndexStride * 3;
		mesh.m_numVertices = numVertices;
		mesh.m_vertexBase = (const unsigned char*)copyBuffer(vertexBase, vertexStride, numVertices);
		mesh.m_vertexStride = vertexStride;
		addIndexedMesh(mesh, triangleIndexStride == 2 ? PHY_SHORT : PHY_INTEGER);

		_managedMeshes[m_instanceID] = this;
	}

	virtual ~btManagedMesh()
	{
		for (unsigned i = 0; i < m_indexedMeshes.size(); ++i)
		{
			delete[](char*)m_indexedMeshes[i].m_vertexBase;
			delete[](char*)m_indexedMeshes[i].m_triangleIndexBase;
		}
		_managedMeshes.erase(m_instanceID);
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

ATTRIBUTE_ALIGNED16(class)
btSingleMesh : public btStridingMeshInterface
{
protected:
	btIndexedMesh m_mesh;
	mutable btVector3 m_aabbMin;
	mutable btVector3 m_aabbMax;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	virtual ~btSingleMesh();

	//just to be backwards compatible
	btSingleMesh(PHY_ScalarType indexType, int numTriangles, void* triangleIndexBase, int triangleIndexStride, PHY_ScalarType vertexType, int numVertices, void* vertexBase, int vertexStride, const Vector3* aabbMin, const Vector3* aabbMax);

	virtual void getLockedVertexIndexBase(unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart = 0);

	virtual void getLockedReadOnlyVertexIndexBase(const unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, const unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart = 0) const;

	/// unLockVertexBase finishes the access to a subpart of the triangle mesh
	/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
	virtual void unLockVertexBase(int subpart) { (void)subpart; }

	virtual void unLockReadOnlyVertexBase(int subpart) const { (void)subpart; }

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices and indices
	virtual int getNumSubParts() const
	{
		return 1;
	}

	virtual void preallocateVertices(int numverts) { (void)numverts; }
	virtual void preallocateIndices(int numindices) { (void)numindices; }

	virtual bool hasPremadeAabb() const;
	virtual void setPremadeAabb(const btVector3& aabbMin, const btVector3& aabbMax) const;
	virtual void getPremadeAabb(btVector3 * aabbMin, btVector3 * aabbMax) const;
};

#endif // _BTMANAGEDMESH_H_

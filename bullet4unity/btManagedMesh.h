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

#endif // _BTMANAGEDMESH_H_

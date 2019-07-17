#ifndef _BTFILTEREDCONVEXRESULTCALLBACK_H_
#define _BTFILTEREDCONVEXRESULTCALLBACK_H_
#include "BulletTypes.h"
#include "UnityTypes.h"

class btFilteredConvexResultCallback : public btCollisionWorld::ConvexResultCallback
{
private:

	btVector3 m_convexFromWorld;  //used to calculate hitPointWorld from hitFraction
	btVector3 m_convexToWorld;

	btVector3 m_hitNormalWorld;
	btVector3 m_hitPointWorld;
	const btCollisionObject* m_hitCollisionObject;

	btCollisionObject** _ignoreList;
	uint _ignoreListSize;
	bool _recordAllHits;
	btAlignedObjectArray<btCollisionWorld::LocalConvexResult> _hits;

public:

	btFilteredConvexResultCallback(const btVector3& convexFromWorld, const btVector3& convexToWorld, btCollisionObject** ignoreList, uint numIgnores, bool all)
		: m_convexFromWorld(convexFromWorld)
		, m_convexToWorld(convexToWorld)
		, m_hitCollisionObject(0)
		, _ignoreList(ignoreList)
		, _ignoreListSize(numIgnores)
		, _recordAllHits(all)
	{
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
	{
		if (!convexResult.m_hitCollisionObject->hasContactResponse())
		{
			return m_closestHitFraction;
		}

		if (0 < _ignoreListSize && _ignoreList)
		{
			for (uint i = 0; i < _ignoreListSize; ++i)
			{
				if (convexResult.m_hitCollisionObject == _ignoreList[i])
				{
					return m_closestHitFraction;
				}
			}
		}

		if (_recordAllHits)
		{
			btCollisionWorld::LocalConvexResult hit(
				convexResult.m_hitCollisionObject,
				convexResult.m_localShapeInfo,
				normalInWorldSpace ? convexResult.m_hitNormalLocal : convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal,
				convexResult.m_hitPointLocal,
				convexResult.m_hitFraction);
			_hits.push_back(hit);
			return m_closestHitFraction;
		}

		//caller already does the filter on the m_closestHitFraction
		btAssert(convexResult.m_hitFraction <= m_closestHitFraction);

		m_closestHitFraction = convexResult.m_hitFraction;
		m_hitCollisionObject = convexResult.m_hitCollisionObject;
		if (normalInWorldSpace)
		{
			m_hitNormalWorld = convexResult.m_hitNormalLocal;
		}
		else
		{
			///need to transform normal into worldspace
			m_hitNormalWorld = m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
		}
		m_hitPointWorld = convexResult.m_hitPointLocal;
		return convexResult.m_hitFraction;
	}

	int getNumHits() const
	{
		return _recordAllHits ? _hits.size() : (hasHit() ? 1 : 0);
	}

	btCollisionData getResult(int index) const
	{
		btCollisionData result;
		memset(&result, 0, sizeof(result));

		if (_recordAllHits)
		{
			result.fraction = _hits[index].m_hitFraction;
			result.collisionObject = (btCollisionObject*)_hits[index].m_hitCollisionObject;
			result.normalWorld = _hits[index].m_hitNormalLocal;
			result.pointWorld = _hits[index].m_hitPointLocal;
		}
		else
		{
			result.fraction = m_closestHitFraction;
			result.collisionObject = (btCollisionObject*)m_hitCollisionObject;
			result.normalWorld = m_hitNormalWorld;
			result.pointWorld = m_hitPointWorld;
		}

		return result;
	}

	const btVector3& getHitPoint(int index) const
	{
		return _recordAllHits ? _hits[index].m_hitPointLocal : m_hitPointWorld;
	}

	const btVector3& getHitNormal(int index) const
	{
		return _recordAllHits ? _hits[index].m_hitNormalLocal : m_hitNormalWorld;
	}

	float getHitFraction(int index) const
	{
		return _recordAllHits ? _hits[index].m_hitFraction : m_closestHitFraction;
	}

};

#endif // _BTFILTEREDCONVEXRESULTCALLBACK_H_

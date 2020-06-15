#ifndef _UNITYTYPES_H_
#define _UNITYTYPES_H_

#ifndef _PINVOKEGENERATOR
	#ifdef _MSC_VER
		#define DLLEXPORT extern "C" __declspec(dllexport) 
	#else
		#define DLLEXPORT extern "C" 
	#endif
	#define DLLAPI __stdcall
    #define STATIC
	#define DESTRUCTOR 
    #define NOEXPORT
    #define CLASSFUNC(CLASSNAME) DLLEXPORT
	#define EXPORT_STRUCT(STRUCTNAME) 
	#define EXPORT_ALIGNED(STRUCTNAME) 
#else
	#define DLLEXPORT 
	#define DLLAPI
	#define DESTRUCTOR __attribute__((annotate("DESTRUCTOR")))
	#define STATIC __attribute__((annotate("STATIC")))
    #define NOEXPORT __attribute__((annotate("NOEXPORT")))
    #define CLASSFUNC(CLASSNAME) __attribute__((annotate("CLASS:"#CLASSNAME)))
	#define EXPORT_STRUCT(STRUCTNAME) static __attribute__((annotate("MANAGED"))) void DUMMY_FUNCTION(STRUCTNAME) {};
	#define EXPORT_ALIGNED(STRUCTNAME) static __attribute__((annotate("ALIGNED"))) void DUMMY_FUNCTION(STRUCTNAME) {};
#endif

template<class T>
inline T* AlignAddressOf(T* ptr)
{
	__int64 addr = (__int64)ptr;
	__int64 offset = (addr % alignof(T));
	return (T*)((char*)ptr + offset);
}

struct NOEXPORT Vector3
{
	float x, y, z;

	struct Marshallizer
	{
		btVector3 value;
		Vector3* ptr;

		Marshallizer(Vector3* src)
		{
			ptr = src;
			value.setValue(src->x, src->y, src->z);
		}

		~Marshallizer()
		{
			ptr->x = value.x();
			ptr->y = value.y();
			ptr->z = value.z();
		}

		operator btVector3& ()
		{
			return value;
		}
	};

	struct ConstMarshallizer
	{
		btVector3 value;

		ConstMarshallizer(const Vector3* src)
		{
			value.setValue(src->x, src->y, src->z);
		}

		operator const btVector3& () const
		{
			return value;
		}
	};

	Vector3()
	{
	}

	Vector3(const btVector3& src)
	{
		x = src.x();
		y = src.y();
		z = src.z();
	}

	Marshallizer Marshall()
	{
		return Marshallizer(this);
	}

	ConstMarshallizer Marshall() const
	{
		return ConstMarshallizer(this);
	}
};

struct NOEXPORT Vector4
{
	float x, y, z, w;

	struct Marshallizer
	{
		btVector4 value;
		Vector4* ptr;

		Marshallizer(Vector4* src)
		{
			ptr = src;
			value.setValue(src->x, src->y, src->z, src->w);
		}

		~Marshallizer()
		{
			ptr->x = value.x();
			ptr->y = value.y();
			ptr->z = value.z();
			ptr->w = value.w();
		}

		operator btVector4& ()
		{
			return value;
		}
	};

	struct ConstMarshallizer
	{
		btVector4 value;

		ConstMarshallizer(const Vector4* src)
		{
			value.setValue(src->x, src->y, src->z, src->w);
		}

		operator const btVector4& () const
		{
			return value;
		}
	};

	Vector4(const btVector4& src)
	{
		x = src.x();
		y = src.y();
		z = src.z();
		w = src.w();
	}

	Marshallizer Marshall()
	{
		return Marshallizer(this);
	}

	ConstMarshallizer Marshall() const
	{
		return ConstMarshallizer(this);
	}
};

struct NOEXPORT Quaternion
{
	float x, y, z, w;

	Quaternion(const btQuaternion& src)
	{
		x = src.x();
		y = src.y();
		z = src.z();
		w = src.w();
	}

	struct Marshallizer
	{
		btQuaternion value;
		Quaternion* ptr;

		Marshallizer(Quaternion* src)
		{
			ptr = src;
			value.setValue(src->x, src->y, src->z);
		}

		~Marshallizer()
		{
			ptr->x = value.x();
			ptr->y = value.y();
			ptr->z = value.z();
			ptr->w = value.w();
		}

		operator btQuaternion& ()
		{
			return value;
		}
	};

	struct ConstMarshallizer
	{
		btQuaternion value;

		ConstMarshallizer(const Quaternion* src)
		{
			value.setValue(src->x, src->y, src->z, src->w);
		}

		operator const btQuaternion& () const
		{
			return value;
		}
	};

	Marshallizer Marshall()
	{
		return Marshallizer(this);
	}

	ConstMarshallizer Marshall() const
	{
		return ConstMarshallizer(this);
	}
};

struct NOEXPORT Matrix4x4
{
	float m[4][4];

	Matrix4x4(const btMatrix3x3& src)
	{
		memset(m, sizeof(m), 0);
		m[0][0] = src[0].x();
		m[0][1] = src[0].y();
		m[0][2] = src[0].z();
		m[1][0] = src[1].x();
		m[1][1] = src[1].y();
		m[1][2] = src[1].z();
		m[2][0] = src[2].x();
		m[2][1] = src[2].y();
		m[2][2] = src[2].z();
		m[3][3] = 1;
	}
};

struct btCollisionData
{
	btCollisionObject* collisionObject;
	float fraction;
	Vector3 normalWorld;
	Vector3 pointWorld;

	btCollisionData() {}
};

struct btPairTestCache
{
	btCollisionObjectArray objects;
};

struct btCollisionObjectBuffer
{
	btCollisionObject objects[1];
};

EXPORT_STRUCT(btCollisionData)

#endif // _UNITYTYPES_H_

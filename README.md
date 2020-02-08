# bullet4unity

This is Bullet Physics wrapper for Unity aimed to support ECS.

Bullet Physics:
  https://github.com/bulletphysics/bullet3

## How it works

Generating project

```
mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" ..
```

bullet4unity and bullet4unity_static are native plugins for Unity. PInvoke codes are generated from PInvokeGenerator.

## Inside

Classes are exposed as unmanaged structs. Every structs implements own interfaces to mimic inheritance.

```cs
public interface IbtCollisionObject : IUnmanagedObject {}

public unsafe struct btCollisionObject : IbtCollisionObject
{
  public IntPtr Ptr { get; set; }

  [Flags]
  public enum AnisotropicFrictionFlags
  {
    CF_ANISOTROPIC_FRICTION_DISABLED = 0,
    CF_ANISOTROPIC_FRICTION = 1,
    CF_ANISOTROPIC_ROLLING_FRICTION = 2,
  }

  public unsafe static btCollisionObject create()
  {
    return new btCollisionObject() { Ptr = (IntPtr)Library.btCollisionObject__create_0() };
  }
}
```

Like Unity's Playable API, class methods are implemented using extension methods.

```cs
public unsafe static bool hasContactResponse<T0>(this T0 self)
  where T0: unmanaged, Bullet.IbtCollisionObject
{
  return Library.btCollisionObject__hasContactResponse_0(self.Ptr);
}

public unsafe static void setCollisionShape<T0, T1>(this T0 self, T1 collisionShape)
  where T0: unmanaged, Bullet.IbtCollisionObject
  where T1: unmanaged, Bullet.IbtCollisionShape
{
  Library.btCollisionObject__setCollisionShape_0(self.Ptr, collisionShape.Ptr);
}

public unsafe static btCollisionShape getCollisionShape<T0>(this T0 self)
  where T0: unmanaged, Bullet.IbtCollisionObject
{
  return new btCollisionShape() { Ptr = (IntPtr)Library.btCollisionObject__getCollisionShape_0(self.Ptr) };
}
```

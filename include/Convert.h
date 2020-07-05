#ifndef BOUNCINGBALL_CONVERT_H
#define BOUNCINGBALL_CONVERT_H

#include <array>

#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec4>
#include <LinearMath/btTransform.h>

namespace toy
{

// bullet_type to(osg_type)
// osg_type    to(bullet_type)

inline osg::Vec3 to(const btVector3& v)
{
    return osg::Vec3(v.x(), v.y(), v.z());
}

inline osg::Vec4 to(const btVector4& v)
{
    return osg::Vec4(v.x(), v.y(), v.z(), v.w());
}

inline btVector3 to(const osg::Vec3& v)
{
    return btVector3(v.x(), v.y(), v.z());
}

inline btVector4 to(const osg::Vec4& v)
{
    return btVector4(v.x(), v.y(), v.z(), v.w());
}

inline osg::Matrix to(const btTransform& v)
{
    auto d = std::array<btScalar, 16>();
    v.getOpenGLMatrix(d.data());

    // Matrix accept both float and double.
    return osg::Matrix(d.data());
}

namespace detail
{

template<typename MatrixType>
inline btTransform getBulletTransform(const MatrixType& v, std::true_type)
{
    auto tf = btTransform();
    tf.setFromOpenGLMatrix(v.ptr());
    return tf;
}

template<typename MatrixType>
inline btTransform getBulletTransform(const MatrixType& v, std::false_type)
{
    auto a = v.ptr();
    auto b = std::array<btScalar, 16>();
    std::copy(a, a + 16, b.data());

    auto tf = btTransform();
    tf.setFromOpenGLMatrix(b.data());
    return tf;
}

}  // namespace detail

inline btTransform to(const osg::Matrix& v)
{
    return detail::getBulletTransform(v, std::is_same<osg::Matrix::value_type, btScalar>());
}

inline btQuaternion to(const osg::Quat& q)
{
    return btQuaternion(q.x(), q.y(), q.z(), q.w());
}

inline osg::Quat to(const btQuaternion& q)
{
    return osg::Quat(q.x(), q.y(), q.z(), q.w());
}

}  // namespace toy

#endif  // BOUNCINGBALL_CONVERT_H

#include <Atom.h>

#include <algorithm>
#include <stdexcept>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <osg/io_utils>

#include <ALSource.h>
#include <BulletClip.h>
#include <Game.h>
#include <Math.h>
#include <MotionState.h>
#include <OsgFactory.h>

namespace toy
{

Atom::Atom()
{
    setName("Atom");
}

Atom::Atom(const Atom& src, const osg::CopyOp& copyop)
    : MatrixTransform(src, copyop)
    , _collisionMask(src._collisionMask)
    , _bodyShape(src._bodyShape)

    // copy callback?
    , _beginCollisionCallbacks(src._beginCollisionCallbacks)
    , _updateCollisionCallbacks(src._updateCollisionCallbacks)
    , _endCollisionCallbacks(src._endCollisionCallbacks)
{
    if (src._motionState)
    {
        setMotionState(std::make_unique<MotionState>(this));
    }

    // are there clone method for bullet?
    // TODO clone btGhostObject, btPairCachingGhostObject ?
    if (src._body)
    {
        auto rigidBody = btRigidBody::upcast(src._body.get());
        if (rigidBody)
        {
            auto body = std::make_unique<btRigidBody>(*rigidBody);
            body->setMotionState(_motionState.get());
            setBody(std::move(body));
        }
        else
        {
            OSG_WARN << "Can't clone non-rigidbody from " << src._name << std::endl;
        }
    }

    // TODO clone other constraints?
}

Atom::~Atom()
{
    if (!_removed)
    {
        OSG_WARN << TOY_HERE << "Unremoved atom found : " << getName() << " " << this
                 << std::endl;
    }
}

void Atom::update(double dt)
{
    updateImplementation(dt);
}

osg::Vec3 Atom::CollisionInfo::getSelfContactPosition(int index) const
{
    auto& point = manifold->getContactPoint(index);
    return to(selfIsA ? point.getPositionWorldOnA() : point.getPositionWorldOnB());
}

osg::Vec3 Atom::CollisionInfo::getAtomContactPosition(int index) const
{
    auto& point = manifold->getContactPoint(index);
    return to(selfIsA ? point.getPositionWorldOnB() : point.getPositionWorldOnA());
}

btRigidBody* Atom::getRigidBody()
{
    return _body ? btRigidBody::upcast(_body.get()) : 0;
}

void Atom::setBody(std::unique_ptr<btCollisionObject> v)
{
    _body = std::move(v);
    _body->setUserPointer(this);
}

void Atom::setBody(std::unique_ptr<btCollisionObject> body,
    std::shared_ptr<btCollisionShape> bodyShape, std::unique_ptr<MotionState> ms)
{
    setBody(std::move(body));
    _bodyShape = move(bodyShape);
    _motionState = move(ms);
}

btRigidBody* Atom::setStaticBody(std::shared_ptr<btCollisionShape> shape)
{
    setBody(sgbc.createStaticBody(shape.get()));
    _bodyShape = std::move(shape);
    return static_cast<btRigidBody*>(_body.get());
}

btRigidBody* Atom::setDynamicBody(std::shared_ptr<btCollisionShape> shape, float mass)
{
    _motionState = std::make_unique<MotionState>(this);
    setBody(sgbc.createDynamicBody(shape.get(), _motionState.get(), mass));
    _bodyShape = std::move(shape);
    return static_cast<btRigidBody*>(_body.get());
}

btRigidBody* Atom::setKinamaticBody(std::shared_ptr<btCollisionShape> shape)
{
    _motionState = std::make_unique<MotionState>(this);
    setBody(sgbc.createKinematicBody(shape.get(), _motionState.get()));
    _bodyShape = std::move(shape);
    return static_cast<btRigidBody*>(_body.get());
}

void Atom::setMotionState(std::unique_ptr<MotionState> v)
{
    _motionState = std::move(v);
}

osg::Vec3 Atom::getWorldPosition() const
{
    return getWorldTransform().getTrans();
}

void Atom::setWorldTransform(const osg::Matrix& m)
{
    if (_body)
    {
        auto frame = to(m);
        _body->setWorldTransform(frame);
        _body->setInterpolationWorldTransform(frame);
    }

    if (getNumParents() > 0)
    {
        auto parentTransform = getParent(0)->getWorldMatrices().front();
        setMatrix(m * osg::Matrix::inverse(parentTransform));
    }
    else
    {
        setMatrix(m);
    }
}

osg::Matrix Atom::getWorldTransform() const
{
    if (_body && !_body->isKinematicObject())
    {
        return to(_body->getWorldTransform());
    }
    else
    {
        return getWorldMatrices().front();
    }
}

void Atom::beginCollision(const CollisionInfo& ci)
{
    for (auto it = _beginCollisionCallbacks.begin(); it != _beginCollisionCallbacks.end();)
    {
        (**(it++))(ci);
    }
}

void Atom::updateCollision(const CollisionInfo& ci)
{
    for (auto it = _updateCollisionCallbacks.begin();
         it != _updateCollisionCallbacks.end();)
    {
        (**(it++))(ci);
    }
}

void Atom::endCollision(const CollisionInfo& ci)
{
    for (auto it = _endCollisionCallbacks.begin(); it != _endCollisionCallbacks.end();)
    {
        (**(it++))(ci);
    }
}

void Atom::addBeginCollisionCallback(CollisionCallbackPtr v)
{
    _beginCollisionCallbacks.push_front(std::move(v));
}

void Atom::removeBeginCollisionCallback(CollisionCallbackPtr v)
{
    _beginCollisionCallbacks.remove(v);
}

Atom::CollisionCallbackList& Atom::getBeginCollisionCallbackList()
{
    return _beginCollisionCallbacks;
}

void Atom::addUpdateCollisionCallback(CollisionCallbackPtr v)
{
    _updateCollisionCallbacks.push_front(std::move(v));
}

void Atom::removeUpdateCollisionCallback(CollisionCallbackPtr v)
{
    _updateCollisionCallbacks.remove(v);
}

Atom::CollisionCallbackList& Atom::getUpdateCollisionCallbackList()
{
    return _updateCollisionCallbacks;
}

void Atom::addEndCollisionCallback(CollisionCallbackPtr v)
{
    _endCollisionCallbacks.push_front(std::move(v));
}

void Atom::removeEndCollisionCallback(CollisionCallbackPtr v)
{
    _endCollisionCallbacks.remove(v);
}

Atom::CollisionCallbackList& Atom::getEndCollisionCallbackList()
{
    return _endCollisionCallbacks;
}

}  // namespace toy

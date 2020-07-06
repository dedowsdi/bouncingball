#include <BulletClip.h>

#include <algorithm>
#include <iostream>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>

#include <Atom.h>
#include <Convert.h>
#include <DebugDrawer.h>
#include <Game.h>
#include <MotionState.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

namespace toy
{
inline Atom* toAtom(const btCollisionObject* co)
{
    return static_cast<Atom*>(co->getUserPointer());
}

void BulletClip::clear()
{
    for (auto i = _world->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        // this should only happens for non atom collision object
        btCollisionObject* obj = _world->getCollisionObjectArray()[i];
        _world->removeCollisionObject(obj);
        delete obj;
    }
}

std::ostream& operator<<(std::ostream& os, const btVector3& v)
{
    os << v.x() << " " << v.y() << " " << v.z();
    return os;
}

void BulletClip::add(btCollisionObject* obj, int collisionGroup, int collisionMask)
{
    auto rigidBody = btRigidBody::upcast(obj);
    if (rigidBody)
        sgbw.addRigidBody(rigidBody, collisionGroup, collisionMask);
    else
        sgbw.addCollisionObject(obj, collisionGroup, collisionMask);
    OSG_INFO << "Add collision object " << obj << " to bullet" << std::endl;
}

void BulletClip::add(btTypedConstraint* v)
{
    sgbw.addConstraint(v);
    OSG_INFO << "Add constraint " << &v << " to bullet" << std::endl;
}

void BulletClip::remove(btCollisionObject* obj)
{
    if (!obj)
    {
        return;
    }

    auto rigidBody = btRigidBody::upcast(obj);
    if (rigidBody)
        _world->removeRigidBody(rigidBody);
    else
        _world->removeCollisionObject(obj);
    OSG_INFO << "Remove collision object " << obj << " from bullet" << std::endl;
}

void BulletClip::remove(btTypedConstraint* v)
{
    if (!v)
    {
        return;
    }

    _world->removeConstraint(v);
    OSG_INFO << "Remove constraint " << v << " from bullet" << std::endl;
}

btCollisionWorld::ClosestRayResultCallback BulletClip::rayTestClosest(
    const osg::Vec3& start, const osg::Vec3& end, int collisionFilterGroup,
    int collisionFilterMask)
{
    auto rayResult = btCollisionWorld::ClosestRayResultCallback(to(start), to(end));
    rayResult.m_collisionFilterMask = collisionFilterMask;
    rayResult.m_collisionFilterGroup = collisionFilterGroup;
    _world->rayTest(to(start), to(end), rayResult);
    return rayResult;
}

btCollisionWorld::ClosestRayResultCallback BulletClip::rayTestSingle(const osg::Vec3& start,
    const osg::Vec3& end, Atom& target, int collisionFilterGroup, int collisionFilterMask)
{
    auto rayResult = btCollisionWorld::ClosestRayResultCallback(to(start), to(end));
    // rayResult.m_flags = 1 << 3;

    auto body = target.getBody();
    if (!body)
    {
        return rayResult;
    }

    auto startTrans = btTransform::getIdentity();
    startTrans.setOrigin(to(start));
    auto endTrans = btTransform::getIdentity();
    endTrans.setOrigin(to(end));

    _world->rayTestSingle(startTrans, endTrans, body, body->getCollisionShape(),
        body->getWorldTransform(), rayResult);

    return rayResult;
}

std::unique_ptr<btRigidBody> BulletClip::createStaticBody(
    btCollisionShape* shape, const osg::Matrix& m)
{
    auto mass = btScalar{0};
    auto localInertia = btVector3(0, 0, 0);
    auto body = std::make_unique<btRigidBody>(mass, nullptr, shape, localInertia);
    body->setWorldTransform(to(m));
    return body;
}

std::unique_ptr<btRigidBody> BulletClip::createKinematicBody(
    btCollisionShape* shape, btMotionState* ms)
{
    auto mass = btScalar{0};
    auto localInertia = btVector3(0, 0, 0);
    auto body = std::make_unique<btRigidBody>(mass, ms, shape, localInertia);
    body->setCollisionFlags(
        body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
    return body;
}

std::unique_ptr<btRigidBody> BulletClip::createDynamicBody(
    btCollisionShape* shape, btMotionState* ms, btScalar mass)
{
    assert(mass > 0);
    auto localInertia = btVector3(0, 0, 0);
    shape->calculateLocalInertia(mass, localInertia);
    auto body = std::make_unique<btRigidBody>(mass, ms, shape, localInertia);
    return body;
}

void BulletClip::beginDebugDrawer()
{
    _debugDrawer->begin();
}

void BulletClip::endDebugDrawer()
{
    _debugDrawer->end();
}

BulletClip::BulletClip()
{
    // init bullet world
    _config.reset(new btDefaultCollisionConfiguration);
    _dispatcher.reset(new btCollisionDispatcher(_config.get()));
    _broadPhase.reset(new btDbvtBroadphase);
    _solver.reset(new btSequentialImpulseConstraintSolver);
    _world.reset(new btDiscreteDynamicsWorld(
        _dispatcher.get(), _broadPhase.get(), _solver.get(), _config.get()));
    _world->setGravity(btVector3(0, 0, -10));

    _ghostPairCallback.reset(new btGhostPairCallback);
    _broadPhase->getOverlappingPairCache()->setInternalGhostPairCallback(
        _ghostPairCallback.get());

    _root = new osg::Group;
    _root->setName("BulletClip");

    _debugDrawer = new DebugDrawer;
    _root->addChild(_debugDrawer);
    _debugDrawer->setNodeMask(0);
    _world->setDebugDrawer(_debugDrawer.get());

    // tick callback
    auto preTick = [](btDynamicsWorld* world, btScalar timeStep) {
        auto bc = static_cast<BulletClip*>(world->getWorldUserInfo());
        bc->preTickCallback(world, timeStep);
    };
    _world->setInternalTickCallback(preTick, this, true);

    auto postTick = [](btDynamicsWorld* world, btScalar timeStep) {
        auto bc = static_cast<BulletClip*>(world->getWorldUserInfo());
        bc->postTickCallback(world, timeStep);
    };
    _world->setInternalTickCallback(postTick, this, false);

    _shootBox = new osg::Geode;

    auto shootBoxMaterial = new osg::Material;
    shootBoxMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0, 0, 1));
    _shootBox->getOrCreateStateSet()->setAttributeAndModes(shootBoxMaterial);

    auto shootBoxGeom = new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1, 1, 1));
    _shootBox->addDrawable(shootBoxGeom);
    shootBoxGeom->setName("ShootBox");
}

BulletClip::~BulletClip() {}

void BulletClip::shootBox(const osg::Vec3& start, const osg::Vec3& dir)
{
    static int index = 0;

    auto box = new Atom();
    box->setName("ShootBox" + std::to_string(index++));
    box->addChild(_shootBox);

    // body
    auto body =
        box->setDynamicBody(std::make_shared<btBoxShape>(btVector3(0.5, 0.5, 0.5)), 1.0f);
    body->setLinearVelocity(to(dir * _shootBoxInitialSpeed));
    body->setCcdMotionThreshold(0.5);
    body->setCcdSweptSphereRadius(0.4f);

    box->setWorldTransform(osg::Matrix::translate(start));

    // debug collisions
    auto beginCollisionCallback = [box, this](const Atom::CollisionInfo& ci) {
        OSG_DEBUG << box->getName() << " start colliding with " << ci.atom->getName()
                   << ", framenumber : " << sgg.getFrameNumber() << std::endl;
    };

    auto updateCollisionCallback = [box, this](const Atom::CollisionInfo& ci) {
        // OSG_DEBUG << box->getName() << " update colliding with " << ci.atom->getName()
        //            << ", framenumber : " << sgg.getFrameNumber() << std::endl;
    };

    auto endCollisionCallback = [box, this](const Atom::CollisionInfo& ci) {
        OSG_DEBUG << box->getName() << " end colliding with " << ci.atom->getName()
                   << ", framenumber : " << sgg.getFrameNumber() << std::endl;
    };

    box->addBeginCollisionCallback(
        std::make_shared<Atom::CollisionCallback>(beginCollisionCallback));
    box->addUpdateCollisionCallback(
        std::make_shared<Atom::CollisionCallback>(updateCollisionCallback));
    box->addEndCollisionCallback(
        std::make_shared<Atom::CollisionCallback>(endCollisionCallback));

    sgg.add(box);
}

void BulletClip::preTickCallback(btDynamicsWorld* world, btScalar timeStep) {}

void BulletClip::postTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
    updateCollisions(timeStep);
}

void BulletClip::updateCollisions(btScalar timeStep)
{
    auto numManifolds = _world->getDispatcher()->getNumManifolds();

    // collect current collisions
    std::set<CollisionPair> currentCollisions;
    for (auto i = 0; i < numManifolds; i++)
    {
        auto contactManifold = _world->getDispatcher()->getManifoldByIndexInternal(i);
        if (contactManifold->getNumContacts() > 0)
        {
            auto bodyA = contactManifold->getBody0();
            auto bodyB = contactManifold->getBody1();

            // be careful here, don't pass temporary value to minmax, it took const T&, not
            // T.
            auto bodyAB = std::minmax(bodyA, bodyB);
            currentCollisions.insert(
                CollisionPair{bodyAB.first, bodyAB.second, contactManifold});
        }
    }

    // dispatch collision begin event
    auto newCollisions = std::set<CollisionPair>{};
    std::set_difference(currentCollisions.begin(), currentCollisions.end(),
        _collisions.begin(), _collisions.end(),
        std::inserter(newCollisions, newCollisions.begin()));

    for (const auto& collision: newCollisions)
    {
        auto atomA = toAtom(collision.objA);
        auto atomB = toAtom(collision.objB);
        atomA->beginCollision({true, timeStep, atomB, collision.manifold});
        atomB->beginCollision({false, timeStep, atomA, collision.manifold});
        OSG_DEBUG << atomA->getName() << " " << atomA << " hit " << atomB->getName() << " "
                   << atomB << std::endl;
    }

    // dispatch collision update event
    auto oldCollisions = std::set<CollisionPair>{};
    std::set_intersection(currentCollisions.begin(), currentCollisions.end(),
        _collisions.begin(), _collisions.end(),
        std::inserter(oldCollisions, oldCollisions.begin()));

    for (const auto& collision: oldCollisions)
    {
        auto atomA = toAtom(collision.objA);
        auto atomB = toAtom(collision.objB);
        atomA->updateCollision({true, timeStep, atomB, collision.manifold});
        atomB->updateCollision({false, timeStep, atomA, collision.manifold});
        // if ( atomA->getType() != Atom::at_static_object &&
        //      atomB->getType() != Atom::at_static_object )
        // {
        //     OSG_DEBUG << atomA->getName() << " update hit " << atomB->getName()
        //                << std::endl;
        // }
    }

    // dispatch collision end event
    auto lostCollisions = std::set<CollisionPair>{};
    std::set_difference(_collisions.begin(), _collisions.end(), currentCollisions.begin(),
        currentCollisions.end(), std::inserter(lostCollisions, lostCollisions.begin()));

    for (const auto& collision: lostCollisions)
    {
        auto atomA = toAtom(collision.objA);
        auto atomB = toAtom(collision.objB);
        atomA->endCollision({true, timeStep, atomB, collision.manifold});
        atomB->endCollision({false, timeStep, atomA, collision.manifold});
        OSG_DEBUG << atomA->getName() << " " << atomA << " lost hit " << atomB->getName()
                   << " " << atomB << std::endl;
    }

    _collisions.swap(currentCollisions);
}

}  // namespace toy

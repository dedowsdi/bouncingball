#ifndef BOUNCINGBALL_BULLETCLIP_H
#define BOUNCINGBALL_BULLETCLIP_H

#include <map>
#include <memory>
#include <set>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec4>

#include <DebugDrawer.h>

#define sgbc ::toy::BulletClip::instance()
#define sgbw ::toy::BulletClip::instance().getWorld()

namespace osgViewer
{
class Viewer;
}

namespace osg
{
class Material;
class Group;
class Geometry;
}  // namespace osg

namespace toy
{
class Atom;

struct CollisionPair
{
    const btCollisionObject* objA;
    const btCollisionObject* objB;
    btPersistentManifold* manifold;
};

inline bool operator<(const CollisionPair& lhs, const CollisionPair& rhs)
{
    return lhs.objA < rhs.objA || (lhs.objA == rhs.objA && lhs.objB < rhs.objB);
};

class DebugDrawer;

class BulletClip
{
public:
    BulletClip(const BulletClip&) = delete;
    BulletClip& operator=(const BulletClip&) = delete;

    static BulletClip& instance()
    {
        static BulletClip* instance = new BulletClip;
        return *instance;
    }

    void clear();

    void add(btCollisionObject* obj, int collisionGroup, int collisionMask);
    void add(btTypedConstraint* v);
    void remove(btCollisionObject* obj);
    void remove(btTypedConstraint* v);

    struct TestRayResult
    {
        Atom* atom;
        osg::Vec3 worldPosition;
    };

    // note, RayResult use 1 as collision filter group by default, this function use -1 by
    // default.
    btCollisionWorld::ClosestRayResultCallback rayTestClosest(const osg::Vec3& start,
        const osg::Vec3& end, int collisionFilterGroup = -1, int collisionFilterMask = -1);

    btCollisionWorld::ClosestRayResultCallback rayTestSingle(const osg::Vec3& start,
        const osg::Vec3& end, Atom& target, int collisionFilterGroup = -1,
        int collisionFilterMask = -1);

    // 0 mass, no motion state
    std::unique_ptr<btRigidBody> createStaticBody(
        btCollisionShape* shape, const osg::Matrix& m = osg::Matrix::identity());

    // 0 mass, CF_KINEMATIC_OBJECT, DISABLE_DEACTIVATION
    std::unique_ptr<btRigidBody> createKinematicBody(
        btCollisionShape* shape, btMotionState* ms);

    // positive mass, with motion state
    std::unique_ptr<btRigidBody> createDynamicBody(
        btCollisionShape* shape, btMotionState* ms, btScalar mass);

    void beginDebugDrawer();

    void endDebugDrawer();

    void shootBox(const osg::Vec3& start, const osg::Vec3& dir);

    btDynamicsWorld& getWorld() const { return *_world.get(); }

    btCollisionConfiguration& getConfig() const { return *_config.get(); }

    btDispatcher& getDispatcher() const { return *_dispatcher.get(); }

    btBroadphaseInterface& getBroadPhase() const { return *_broadPhase.get(); }

    btConstraintSolver& getSolver() const { return *_solver.get(); }

    osg::Group* getDebugRoot() const { return _root; }
    void setDebugRoot(osg::Group* v) { _root = v; }

    DebugDrawer* getDebugDrawer() const { return _debugDrawer; }
    void setDebugDrawer(DebugDrawer* v) { _debugDrawer = v; }

    float getShootBoxInitialSpeed() const { return _shootBoxInitialSpeed; }
    void setShootBoxInitialSpeed(float v) { _shootBoxInitialSpeed = v; }

private:
    BulletClip();
    ~BulletClip();

    void preTickCallback(btDynamicsWorld* world, btScalar timeStep);
    void postTickCallback(btDynamicsWorld* world, btScalar timeStep);

    // TODO move this function to Game, get Game stuff out of BulletClip?
    void updateCollisions(btScalar timeStep);

    float _shootBoxInitialSpeed = 256.0f;

    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osg::Geode> _shootBox;
    osg::ref_ptr<DebugDrawer> _debugDrawer;

    std::unique_ptr<btDynamicsWorld> _world;
    std::unique_ptr<btCollisionConfiguration> _config;
    std::unique_ptr<btDispatcher> _dispatcher;
    std::unique_ptr<btBroadphaseInterface> _broadPhase;
    std::unique_ptr<btConstraintSolver> _solver;
    std::unique_ptr<btGhostPairCallback> _ghostPairCallback;

    std::set<CollisionPair> _collisions;
};

}  // namespace toy

#endif  // BOUNCINGBALL_BULLETCLIP_H

#ifndef BOUNCINGBALL_ATOM_H
#define BOUNCINGBALL_ATOM_H

#include <functional>
#include <memory>
#include <forward_list>

#include <osg/MatrixTransform>

class btManifoldPoint;
class btCollisionObject;
class btRigidBody;
class btTypedConstraint;
class btPersistentManifold;
class btCollisionShape;

namespace toy
{

class ALSource;
class MotionState;
class Atom;

// A scene object that have optional physic body.
//
// Life time:
//      Game::add(...) add new Atom to osg and bullet directly. Some bullet operation can
//      only be called after added to world. If your new atom is a child of SceneRoot, you
//      should never call this in any traversal of descendant of SceneRoot, otherwise you
//      are screwed when the +children cause SceneRoot to reallocate memory.
//
//      You can safely call Game::add(...) in update(dt), it's in update traversal of parent
//      of SceneRoot.
//
//      Game::remove(...) remove Atom from osg and bullet, it's delayed, the actual remove
//      happnens at the end of Game::update. You can call it anytime at any place.
//
//      Remove collisiion object would remove contact info, imagine you remove an Atom in a
//      begin collision callback, but this atom is colliding with another Atom, the second
//      atom will lost contact info(no points in btPersistentManifold any more) if you
//      remove it directly. That's why it's removed in the end of Game::update.
//
//      Removed Atom are dying zombie, it's not not released immediately, Game keep a
//      reference to it for a few frames, it's released if Game clear it's reference and no
//      other reference exists.
//
// Transform:
//      kinematic  osg -> bullet
//      dynamic    bullet -> osg
//      static     independent
//
//      All Atom without parent will be child of Game::_sceneRoot when it's added to Game.
//
// Sound:
//      When you play a sound, the sound moves with this atom, it's removed automatically
//      after it's buffer duration if it's loop is false.
//
class Atom : public osg::MatrixTransform
{
public:
    Atom();

    Atom(const Atom& src, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    META_Node(toy, Atom);

    virtual ~Atom();

    virtual void update(double dt);

    struct CollisionInfo
    {
        // self is the atom that calls the collision callback
        bool selfIsA;
        double deltaTime;  // only used for update
        Atom* atom;
        btPersistentManifold* manifold;

        osg::Vec3 getSelfContactPosition(int index = 0) const;
        osg::Vec3 getAtomContactPosition(int index = 0) const;
    };

    virtual btRigidBody* getRigidBody();

    btCollisionObject* getBody() { return _body.get(); }
    const btCollisionObject* getBody() const { return _body.get(); }
    void setBody(std::unique_ptr<btCollisionObject> v);

    void setBody(std::unique_ptr<btCollisionObject> body,
        std::shared_ptr<btCollisionShape> bodyShape,
        std::unique_ptr<MotionState> ms = nullptr);

    btRigidBody* setStaticBody(std::shared_ptr<btCollisionShape> shape);

    btRigidBody* setDynamicBody(std::shared_ptr<btCollisionShape> shape, float mass);

    btRigidBody* setKinamaticBody(std::shared_ptr<btCollisionShape> shape);

    std::shared_ptr<btCollisionShape> getBodyShape() const { return _bodyShape; }
    void setBodyShape(std::shared_ptr<btCollisionShape> v) { _bodyShape = std::move(v); }

    MotionState* getMotionState() { return _motionState.get(); }
    const MotionState* getMotionState() const { return _motionState.get(); }
    void setMotionState(std::unique_ptr<MotionState> v);

    osg::Vec3 getWorldPosition() const;

    // set both graph and body transform
    void setWorldTransform(const osg::Matrix& m);
    osg::Matrix getWorldTransform() const;

    using CollisionCallback = std::function<void(const CollisionInfo& ci)>;
    using CollisionCallbackPtr = std::shared_ptr<CollisionCallback>;
    using CollisionCallbackList = std::forward_list<CollisionCallbackPtr>;

    virtual void beginCollision(const CollisionInfo& ci);
    virtual void updateCollision(const CollisionInfo& ci);
    virtual void endCollision(const CollisionInfo& ci);

    void addBeginCollisionCallback(CollisionCallbackPtr v);
    void removeBeginCollisionCallback(CollisionCallbackPtr v);
    CollisionCallbackList& getBeginCollisionCallbackList();

    void addUpdateCollisionCallback(CollisionCallbackPtr v);
    void removeUpdateCollisionCallback(CollisionCallbackPtr v);
    CollisionCallbackList& getUpdateCollisionCallbackList();

    void addEndCollisionCallback(CollisionCallbackPtr v);
    void removeEndCollisionCallback(CollisionCallbackPtr v);
    CollisionCallbackList& getEndCollisionCallbackList();

    bool getRemoved() const { return _removed; }
    void setRemoved(bool v) { _removed = v; }

    unsigned getCollisionMask() const { return _collisionMask; }
    void setCollisionMask(unsigned v) { _collisionMask = v; }

    unsigned getCollisionGroup() const { return _collisionGroup; }
    void setCollisionGroup(unsigned v) { _collisionGroup = v; }

protected:
    virtual void updateImplementation(double dt){};

    bool _removed = false;
    unsigned _collisionMask = -1;
    unsigned _collisionGroup = -1;

    std::unique_ptr<btCollisionObject> _body;
    std::shared_ptr<btCollisionShape> _bodyShape;  // same as _body->_m_colisionShape
    std::unique_ptr<MotionState> _motionState;

    CollisionCallbackList _beginCollisionCallbacks;
    CollisionCallbackList _updateCollisionCallbacks;
    CollisionCallbackList _endCollisionCallbacks;
};

}  // namespace toy

#endif  // BOUNCINGBALL_ATOM_H

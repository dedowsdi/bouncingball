#ifndef BOUNCINGBALL_GAME_H
#define BOUNCINGBALL_GAME_H

#include <memory>
#include <set>
#include <vector>

#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>

#define TOY_STR(x) TOY_STR2(x)
#define TOY_STR2(x) #x
#define TOY_HERE (__FILE__ ":" TOY_STR(__LINE__) ":")

namespace osg
{
class Group;
class Vec2i;
class Texture2D;
class Camera;
class Geometry;
class Program;
class Shader;
}  // namespace osg

namespace osgGA
{
class CameraManipulator;
}

namespace osgViewer
{
class Viewer;
};

namespace toy
{

class Atom;
class ALBuffer;
class ALSource;
class ALContext;
class ALListener;

#define sgg ::toy::Game::instance()

// Singleton.
class Game
{
public:
    Game(const Game&) = delete;

    Game& operator=(const Game&) = delete;

    static Game& instance()
    {
        static Game instance;
        return instance;
    }

    void clear();

    void init(int argc, char* argv[], osgViewer::Viewer* viewer);

    // see Atom for explanation
    void add(Atom* v);

    // see Atom for explanation
    void remove(Atom* v);

    void update(double dt);

    // Root children is fixed, don't add or remove anything to it during gaming.
    // If you want to manipulate children of _sceneRoot, make sure it happens in
    // update or event traversal of root.
    osg::Group* getRoot() { return _root; }

    osg::Group* getDebugRoot() { return _debugRoot; }

    osg::Vec2i getWindowSize();

    osg::Vec2 getWindowCenter();

    void debugDrawLine(const osg::Vec3& from, const osg::Vec3& to,
        const osg::Vec4& fromColor, const osg::Vec4& toColor);

    void debugDrawSphere(const osg::Vec3& pos, float radius, const osg::Vec4& color);

    int getZombieFrames() const { return _zombies.size(); }
    void setZombieFrames(int v);

    bool getPaused() const { return _paused; }
    void setPaused(bool v);

    int getDebugFrames() const { return _debugFrames; }
    void setDebugFrames(int v) { _debugFrames = v; }

    bool getDebugging() const { return _debugging; }
    void setDebugging(bool v);

    int getFrameNumber() const { return _frameNumber; }

    double getDeltaTime() const { return _deltaTime; }

    osg::Camera* getMainCamera() const;

    osgViewer::Viewer* getViewer() { return _viewer; }
    const osgViewer::Viewer* getViewer() const { return _viewer; }
    void setViewer(osgViewer::Viewer* v) { _viewer = v; }

    osg::Group* getSceneRoot() const { return _sceneRoot; }
    void setSceneRoot(osg::Group* v) { _sceneRoot = v; }

    ALListener* getEar() const { return _alListener; }
    ALListener* getALListener() const { return _alListener; }
    void setALListener(ALListener* v);

private:
    Game();

    ~Game();

    void updatePhyscis();

    void updateAtoms();

    void clearDeadAtoms();

    void addNewAtoms();

    void removeAtomImplemantatoin(Atom* atom);

    void createScene();

    void createSound(int argc, char* argv[]);

    void createRoots();

    void createALBuffers();

    void createALSources();

    // declare ALContext as first member, init it before everything, clear it
    // after everything.(as long as you don't use static or global object)
    std::unique_ptr<ALContext> _alContext;

    bool _debugging = false;
    bool _paused = false;

    // updated times, less than FrameStamp if ever paused
    int _frameNumber = 0;
    int _debugFrames = 0;
    double _deltaTime = 0;

    osg::Vec3 _sun = osg::Vec3(0, 0, 1);

    osgViewer::Viewer* _viewer = 0;

    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osg::Group> _sceneRoot;
    osg::ref_ptr<osg::Group> _sceneRoot2;
    osg::ref_ptr<osg::Group> _debugRoot;

    osg::ref_ptr<osg::Geometry> _debugLines;

    osg::ref_ptr<ALListener> _alListener;

    using AtomSet = std::set<Atom*>;
    using AtomRefSet = std::set<osg::ref_ptr<Atom>>;
    AtomSet _atoms;
    AtomSet _newAtoms;
    AtomSet _removedAtoms;
    std::vector<AtomRefSet> _zombies;
};

}  // namespace toy

#endif  // BOUNCINGBALL_GAME_H

#include <Game.h>

#include <cassert>

#include <osg/AnimationPath>
#include <osg/Billboard>
#include <osg/BlendFunc>
#include <osg/ClipPlane>
#include <osg/Hint>
#include <osg/LightModel>
#include <osg/LineWidth>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgViewer/Viewer>

#include <ALBuffer.h>
#include <ALContext.h>
#include <ALListener.h>
#include <ALSource.h>
#include <Atom.h>
#include <BulletClip.h>
#include <Math.h>
#include <MotionState.h>
#include <OsgFactory.h>
#include <OsgQuery.h>

namespace toy
{

class GameUpdater : public osg::Callback
{
    bool run(osg::Object* object, osg::Object* data) override
    {
        static auto time0 = data->asNodeVisitor()->getFrameStamp()->getReferenceTime();
        auto time1 = data->asNodeVisitor()->getFrameStamp()->getReferenceTime();
        sgg.update(time1 - time0);
        time0 = time1;
        return traverse(object, data);
    }
};

void Game::clear()
{
    auto atoms = _atoms;
    for (auto atom: atoms)
    {
        removeAtomImplemantatoin(atom);
    }
    _atoms.clear();
    _zombies.clear();
    _newAtoms.clear();
    _removedAtoms.clear();
}

void Game::init(int argc, char* argv[], osgViewer::Viewer* viewer)
{
    _viewer = viewer;
    _viewer->realize();

    setZombieFrames(4);

    createScene();
    _viewer->setSceneData(_root);

    createSound(argc, argv);

    _root->addUpdateCallback(new GameUpdater());
}

void Game::add(Atom* v)
{
    auto iter = _atoms.find(v);
    if (iter != _atoms.end())
    {
        OSG_NOTICE << "Ignore " << v->getName() << ", it's already added" << std::endl;
        return;
    }

    _newAtoms.insert(v);
    addNewAtoms();
}

void Game::remove(Atom* v)
{
    // delay removing, make sure no atom removed during current frame BulletClip
    // collision dispatch.
    _removedAtoms.insert(v);
}

void Game::update(double dt)
{
    _deltaTime = dt;

    // In order to reserve debug draw during updateAtoms, no beginDebugDrawer
    // should be called during pause, note that you must turn on debugging
    // before pause to see the debug draw.
    if (_paused && --_debugFrames < 0)
        return;

    ++_frameNumber;

    if (_debugging)
        sgbc.beginDebugDrawer();

    updatePhyscis();

    clearDeadAtoms();

    updateAtoms();

    // Add new atoms  after update existing ones, new atoms should not be
    // updated this frame. addNewAtoms();

    if (_debugging)
    {
        sgbw.debugDrawWorld();
        sgbc.endDebugDrawer();
    }
}

osg::Vec2i Game::getWindowSize()
{
    auto rect = osgt::getWindowRect(*_viewer);
    return osg::Vec2i(rect.z(), rect.w());
}

osg::Vec2 Game::getWindowCenter()
{
    auto size = getWindowSize();
    return osg::Vec2(size.x() * 0.5f, size.y() * 0.5f);
}

void Game::debugDrawLine(const osg::Vec3& from, const osg::Vec3& to,
    const osg::Vec4& fromColor, const osg::Vec4& toColor)
{
    if (!_debugLines)
    {
        _debugLines = new osg::Geometry;
        auto vertices = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        auto colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        _debugLines->setVertexArray(vertices);
        _debugLines->setColorArray(colors);
        _debugLines->setDataVariance(osg::Object::DYNAMIC);

        _debugLines->setName("Game#DebugLines");
        _debugLines->setUseDisplayList(false);
        _debugLines->setUseVertexBufferObjects(true);
        _debugLines->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));

        _debugRoot->addChild(_debugLines);
    }

    auto vertices = static_cast<osg::Vec3Array*>(_debugLines->getVertexArray());
    auto colors = static_cast<osg::Vec4Array*>(_debugLines->getColorArray());
    vertices->push_back(from);
    vertices->push_back(to);
    colors->push_back(fromColor);
    colors->push_back(toColor);

    vertices->dirty();
    colors->dirty();

    static_cast<osg::DrawArrays*>(_debugLines->getPrimitiveSet(0))
        ->setCount(vertices->size());
    _debugLines->dirtyBound();
}

void Game::debugDrawSphere(const osg::Vec3& pos, float radius, const osg::Vec4& color)
{
    _debugRoot->addChild(osgf::createSphereAt(pos, radius, color));
}

// Don't inline this, resize requires complete type osg::Fog toy::Atom
void Game::setZombieFrames(int v)
{
    _zombies.resize(v);
}

void Game::setDebugging(bool v)
{
    _debugging = v;
    if (_debugging)
    {
        sgbc.getDebugDrawer()->setNodeMask(-1);
    }
    else
    {
        sgbc.getDebugDrawer()->setNodeMask(0);
    }
    OSG_NOTICE << "set debugging to " << _debugging << std::endl;
}

osg::Camera* Game::getMainCamera() const
{
    return _viewer->getCamera();
}

void Game::setALListener(ALListener* v)
{
    _alListener = v;
}

void Game::setPaused(bool v)
{
    _paused = v;
    _debugFrames = 0;
}

Game::Game() {}

Game::~Game() {}

void Game::updatePhyscis()
{
    sgbw.stepSimulation(_deltaTime);
}

void Game::clearDeadAtoms()
{
    auto& zombies = _zombies[_frameNumber % getZombieFrames()];

#ifdef DEBUG
    if (!zombies.empty())
    {
        for (const auto& item: zombies)
        {
            OSG_NOTICE << "remove dead atom " << item->getName() << " at frame "
                       << _frameNumber << "\n";
        }
        OSG_NOTICE << std::endl;
    }
#endif /* ifndef  */

    zombies.clear();

    for (auto atom: _removedAtoms)
    {
        zombies.insert(atom);
        removeAtomImplemantatoin(atom);
    }
    _removedAtoms.clear();
}

void Game::addNewAtoms()
{
    for (auto atom: _newAtoms)
    {
        OSG_NOTICE << "Add " << atom->getName() << " " << atom << " to game" << std::endl;
        atom->setRemoved(false);
        _atoms.insert(atom);

        if (atom->getNumParents() == 0)
        {
            _sceneRoot->addChild(atom);
        }

        auto body = atom->getBody();
        if (body)
        {
            assert(body->getCollisionShape() == atom->getBodyShape().get());
            sgbc.add(body, atom->getCollisionGroup(), atom->getCollisionMask());
        }
    }
    _newAtoms.clear();
}

void Game::updateAtoms()
{
    for (auto atom: _atoms)
    {
        atom->update(_deltaTime);
    }
}

void Game::removeAtomImplemantatoin(Atom* atom)
{
    OSG_NOTICE << "Remove " << atom->getName() << std::endl;

    _atoms.erase(atom);
    atom->setRemoved(true);

    // remove physics
    auto body = atom->getBody();
    if (body)
    {
        sgbc.remove(body);
    }

    // remove graph
    auto numParents = atom->getNumParents();
    for (auto i = numParents; i > 0; --i)
    {
        atom->getParent(i - 1)->removeChild(atom);
    }
}

void Game::createScene()
{
    createRoots();
}

void Game::createSound(int argc, char* argv[])
{
    _alContext.reset(new ALContext(argc, argv));
    _alListener = new ALListener;

    // make sure listener is not updated before sources
    auto listenerUpdater = new ALListenerUpdater(_alListener);
    _sceneRoot2->addUpdateCallback(listenerUpdater);

    OSG_NOTICE << "OpenAL init finished.\n" << std::string(80, '*') << std::endl;
}

void Game::createRoots()
{
    _root = new osg::Group();
    _root->setName("Game");

    _debugRoot = new osg::Group();
    _debugRoot->setName("DebugRoot");
    _debugRoot->addChild(sgbc.getDebugRoot());

    _root->addChild(_debugRoot);

    _sceneRoot = new osg::Group();
    _sceneRoot->setName("SceneRoot");

    _root->addChild(_sceneRoot);

    _sceneRoot2 = new osg::Group();
    _sceneRoot2->setName("SceneRoot2");

    _root->addChild(_sceneRoot2);
}

}  // namespace toy

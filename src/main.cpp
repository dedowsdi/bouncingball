// A ball bouncing on a cube, play sound and change color when hit.

#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>
#include <bullet/BulletCollision/CollisionShapes/btSphereShape.h>
#include <osg/LightModel>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osgDB/ReaderWriter>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <ALListener.h>
#include <ALSource.h>
#include <Atom.h>
#include <BulletClip.h>
#include <Convert.h>
#include <DB.h>
#include <DebugHandler.h>
#include <Game.h>
#include <Math.h>
#include <OsgFactory.h>

void createScene()
{
    using namespace toy;

    // add cube as ground
    auto ground = new Atom();
    ground->setName("Ground");
    {
        // graph
        auto size = osg::Vec3(10, 10, 1);
        auto box = new osg::Box(osg::Vec3(), size.x(), size.y(), size.z());
        auto graph = new osg::ShapeDrawable(box);
        ground->addChild(graph);

        // body
        auto pyShape = std::make_shared<btBoxShape>(to(size * 0.5f));
        auto body = ground->setStaticBody(pyShape);
        body->setRestitution(1.0);
        body->setContactStiffnessAndDamping(BT_LARGE_FLOAT, 0);
    }
    sgg.add(ground);

    // add bouncing ball
    auto ball = new Atom();
    ball->setName("Ball");
    {
        // graph
        auto size = 1.0f;
        auto sphere = new osg::Sphere(osg::Vec3(), size);
        auto graph = new osg::ShapeDrawable(sphere);

        auto hint = new osg::TessellationHints;
        hint->setDetailRatio(4.0);
        graph->setTessellationHints(hint);

        auto mtl = new osg::Material;
        mtl->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
        mtl->setShininess(osg::Material::FRONT_AND_BACK, 80);
        mtl->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);

        graph->getOrCreateStateSet()->setAttributeAndModes(mtl);

        ball->addChild(graph);

        // body
        auto pyShape = std::make_shared<btSphereShape>(size);
        auto body = ball->setDynamicBody(pyShape, 1.0f);
        body->setRestitution(1.0);
        body->setContactStiffnessAndDamping(BT_LARGE_FLOAT, 0);
        ball->setWorldTransform(osg::Matrix::translate(osg::Vec3(0, 0, 4)));

        // hit callback
        auto callback = std::make_shared<Atom::CollisionCallback>(
            [=](const Atom::CollisionInfo& ci) -> void {
                graph->setColor(toy::linearRand(osg::Vec4(), osg::Vec4(1, 1, 1, 1)));
                auto sound = new toy::ALSource(osgDB::readALBufferFile("sound/hit.wav"));
                toy::playSound(ball, sound);
            });
        ball->addBeginCollisionCallback(callback);
    }
    sgg.add(ball);
}

int main(int argc, char* argv[])
{
    srand(time(0));

    // cache everything
    auto options = new osgDB::Options();
    options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    osgDB::Registry::instance()->setOptions(options);

    osgViewer::Viewer viewer;
    sgg.init(argc, argv, &viewer);

    createScene();

    auto camera = viewer.getCamera();
    camera->setName("MainCamera");
    camera->addChild(sgg.getEar());

    auto lm = new osg::LightModel;
    lm->setLocalViewer(true);

    camera->getOrCreateStateSet()->setAttributeAndModes(lm);

    // Add some debug handlers:
    // f1   Stat
    // f2   Print scene
    // f3   Print render stages
    // f4   Save main camera to main.osgt
    // f5   Toggle bullet debug draw
    // f6   Toggle bullet draw life time
    // f7   Toggle bullet draw normal
    // f8   Shoot box
    // f9   Pause/Continue
    // f10  Forward 1 frame
    // +    +10 shoot box initial speed
    // -    -10 shoot box initial speed
    // 7    Toggle lighting
    // 8    Toggle back face culling
    // 9    Toggle texturing
    // 0    Cycle polygon mode
    viewer.addEventHandler(new toy::DebugHandler(viewer.getCamera()));

    auto statsHandler = new osgViewer::StatsHandler;
    statsHandler->setKeyEventTogglesOnScreenStats(osgGA::GUIEventAdapter::KEY_F1);

    viewer.addEventHandler(statsHandler);

    auto statesetHandler =
        new osgGA::StateSetManipulator(sgg.getRoot()->getOrCreateStateSet());
    statesetHandler->setKeyEventToggleLighting('7');
    statesetHandler->setKeyEventToggleBackfaceCulling('8');
    statesetHandler->setKeyEventToggleTexturing('9');
    statesetHandler->setKeyEventCyclePolygonMode('0');

    viewer.addEventHandler(statesetHandler);

    viewer.run();

    sgg.clear();

    return 0;
}

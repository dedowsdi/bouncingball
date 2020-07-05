#include <algorithm>
#include <sstream>

#include <osg/Geode>
#include <osg/Notify>
#include <osgText/Text>
#include <Convert.h>
#include <DebugDrawer.h>

namespace
{

osg::Geometry* createGeometry()
{
    auto geom = new osg::Geometry;
    auto vertices = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    auto colors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setDataVariance(osg::Object::DYNAMIC);

    return geom;
}

inline osg::Vec3Array* getVertices(osg::Geometry* geom)
{
    return static_cast<osg::Vec3Array*>(geom->getVertexArray());
}

inline osg::Vec3Array* getColors(osg::Geometry* geom)
{
    return static_cast<osg::Vec3Array*>(geom->getColorArray());
}

void updatePrimitiveSet(osg::Geometry* geom, GLenum mode)
{
    auto vertices = getVertices(geom);
    if (vertices->size() > 0)
        geom->addPrimitiveSet(new osg::DrawArrays(mode, 0, vertices->size()));
}

}  // namespace

namespace toy
{

DebugDrawer::DebugDrawer()
{
    setName("DebugDrawer");

    _leaf = new osg::Geode;
    auto ss = _leaf->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    ss->setRenderBinDetails(99, "RenderBin");

    _points = createGeometry();
    _points->setName("DebugDrawer#Points");
    _leaf->addDrawable(_points);

    _lines = createGeometry();
    _lines->setName("DebugDrawer#Lines");
    _leaf->addDrawable(_lines);

    addChild(_leaf);

    // _triangles = createGeometry();
    // _triangles->setName("DebugDrawer_triangles");
    // _leaf->addDrawable( _triangles );

    _font = osgText::readFontFile("fonts/arial.ttf");

    _textPool.reserve(_textPoolSize);
    std::generate_n(
        std::back_inserter(_textPool), _textPoolSize, [this]() { return createText(); });
}

void DebugDrawer::drawLine(const btVector3& fromPoint, const btVector3& toPoint,
    const btVector3& fromColor, const btVector3& toColor)
{
    auto osgFromPoint = to(fromPoint);
    auto osgToPoint = to(toPoint);

    // no endless line
    if (osgFromPoint.length2() >= 100000000 || osgToPoint.length2() >= 100000000)
    {
        return;
    }

    auto vertices = getVertices(_lines);
    auto colors = getColors(_lines);
    vertices->push_back(osgFromPoint);
    vertices->push_back(osgToPoint);
    colors->push_back(to(fromColor));
    colors->push_back(to(toColor));
}

void DebugDrawer::drawLine(
    const btVector3& fromPoint, const btVector3& toPoint, const btVector3& color)
{
    drawLine(fromPoint, toPoint, color, color);
}

void DebugDrawer::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
    btScalar distance, int lifeTime, const btVector3& color)
{
    auto vertices = getVertices(_points);
    auto colors = getColors(_points);
    vertices->push_back(to(PointOnB));
    colors->push_back(to(color));

    // distance can be very small, most of time you won't see it.
    if (_drawContactNormal)
        drawLine(PointOnB, PointOnB + normalOnB * distance, color);

    std::stringstream ss;
    ss << " " << lifeTime;

    if (_drawCotactLifeTime)
        draw3dText(PointOnB, ss.str().c_str());
}

void DebugDrawer::reportErrorWarning(const char* warningString)
{
    OSG_INFO << warningString << std::endl;
}

void DebugDrawer::draw3dText(const btVector3& location, const char* textString)
{
    if (_numTexts == _textPool.size())
    {
        _textPool.reserve(_textPool.size() + 4);
        std::generate_n(
            std::back_inserter(_textPool), 4, [this]() { return createText(); });
    }

    auto text = _textPool[_numTexts];
    text->setPosition(to(location));
    text->setText(textString);

    _leaf->addDrawable(text);

    ++_numTexts;
}

void clearGeometry(osg::Geometry* geom)
{
    getVertices(geom)->clear();
    getColors(geom)->clear();
    geom->removePrimitiveSet(0, geom->getNumPrimitiveSets());
}

void DebugDrawer::begin()
{
    clearGeometry(_points);
    clearGeometry(_lines);
    // clearGeometry( _triangles );

    if (_leaf->getNumDrawables() > 3)
        _leaf->removeDrawables(3, _leaf->getNumDrawables() - 3);

    _numTexts = 0;
}

void DebugDrawer::end()
{
    updatePrimitiveSet(_points, GL_POINTS);
    updatePrimitiveSet(_lines, GL_LINES);
    // updatePrimitiveSet( _triangles, GL_TRIANGLES );
}

osgText::Text* DebugDrawer::createText()
{
    auto text = new osgText::Text;
    text->setDataVariance(osg::Object::DYNAMIC);
    text->setCharacterSize(_characterSize);
    text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text->setAxisAlignment(osgText::TextBase::SCREEN);
    text->setFont("fonts/arial.ttf");

    return text;
}

}  // namespace toy

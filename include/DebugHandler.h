#ifndef BOUNCINGBALL_DEBUGHANDLER_H
#define BOUNCINGBALL_DEBUGHANDLER_H

#include <stack>
#include <string>

#include <osgDB/WriteFile>
#include <osgGA/GUIEventHandler>
#include <osgUtil/PrintVisitor>
#include <osgUtil/RenderStage>
#include <osgUtil/StateGraph>
#include <osgViewer/Renderer>

namespace toy
{

// Recursively print render leaves.
class RenderStagePrinter : public osgUtil::RenderBin::DrawCallback
{
public:
    RenderStagePrinter(std::ostream& out);

    void drawImplementation(osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo,
        osgUtil::RenderLeaf*& previous) override;

    bool getEnabled() const { return _enabled; }
    void setEnabled(bool v) { _enabled = v; }

private:
    struct OrderedRenderStage
    {
        int renderOrder;  // -1 : pre , 0 : normal, 1 : post
        int order;        // The order in RenderStage::RenderStageOrderPair
        const osgUtil::RenderStage* stage;
    };

    using RenderBinStack = std::vector<const osgUtil::RenderBin*>;
    using RenderStageStack = std::vector<OrderedRenderStage>;

    void printRenderStage(const osgUtil::RenderStage* stage);

    void pushRenderStage(int renderOrder, int order, const osgUtil::RenderStage* stage);
    void popRenderStage();

    void pushRenderBin(const osgUtil::RenderBin* bin);
    void popRenderBin();

    void printRenderBin(const osgUtil::RenderBin* bin);

    void printPath();

    template<typename T>
    void printLeaves(const T& leaves) const
    {
        for (const auto leaf: leaves)
        {
            auto drawable = leaf->getDrawable();
            _out << "    " << drawable << " \"" << drawable->getName() << " "
                 << (leaf->_dynamic ? "DYNAMIC" : "STATIC") << "\n";
        }
    }

    mutable bool _enabled = false;
    std::ostream& _out;

    RenderStageStack _stages;
    RenderBinStack _bins;
};

class VerbosePrintVisitor : public osgUtil::PrintVisitor
{
public:
    using PrintVisitor::PrintVisitor;

    void apply(osg::Node& node) override;
};

class DebugHandler : public osgGA::GUIEventHandler
{
public:
    DebugHandler(osg::Camera* camera);

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

    void setCamera(osg::Camera* v);

private:
    osg::Camera* _camera = 0;
    osg::ref_ptr<RenderStagePrinter> _renderStagePrinter;
};

}  // namespace toy
#endif  // BOUNCINGBALL_DEBUGHANDLER_H

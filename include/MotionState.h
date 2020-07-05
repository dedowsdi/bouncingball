#ifndef BOUNCINGBALL_MOTIONSTATE_H
#define BOUNCINGBALL_MOTIONSTATE_H

#include <bullet/LinearMath/btMotionState.h>

#include <osg/MatrixTransform>

#include <Convert.h>

namespace toy
{

// Assume center of mass is center of graph.
// TODO cach matrix transform.
class MotionState : public btMotionState
{

public:
    MotionState(osg::MatrixTransform* transform) : _transform(transform) {}

    void getWorldTransform(btTransform& worldTrans) const override
    {
        auto nodes = _transform->getParentalNodePaths();
        worldTrans = to(osg::computeLocalToWorld(nodes.front()));
    }

    void setWorldTransform(const btTransform& worldTrans) override
    {
        auto parentTransform = _transform->getParent(0)->getWorldMatrices().front();
        _transform->setMatrix(to(worldTrans) * osg::Matrix::inverse(parentTransform));
    }

private:
    osg::MatrixTransform* _transform;
};

}  // namespace toy
#endif  // BOUNCINGBALL_MOTIONSTATE_H

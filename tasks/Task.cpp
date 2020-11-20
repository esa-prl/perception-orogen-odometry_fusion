/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include "base/samples/RigidBodyState.hpp"

using namespace std;
using namespace odometry_fusion;
using namespace Eigen;
using namespace base::samples;

IOFormat singleLine(StreamPrecision, DontAlignCols, ",\t", ";\t", "", "", "[", "]");

Task::Task(string const& name) : TaskBase(name), library(new OdometryFusion()) {}

Task::~Task() { delete library; }

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook()) return false;
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook()) return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    RigidBodyState delta_pose;
    if (_inertial_delta_pose_in.read(delta_pose) == RTT::NewData)
    {
        processInertialOdometryIn(delta_pose);
    }

    if (_visual_delta_pose_in.read(delta_pose) == RTT::NewData)
    {
        processVisualOdometryIn(delta_pose);
    }
}
void Task::processVisualOdometryIn(RigidBodyState delta_pose)
{
    cout << "[VISUAL]" << z.format(singleLine) << endl;
}
void Task::processInertialOdometryIn(RigidBodyState delta_pose)
{
    cout << "[INERTIAL]" << delta_pose.position << endl;
}
void Task::errorHook() { TaskBase::errorHook(); }
void Task::stopHook() { TaskBase::stopHook(); }
void Task::cleanupHook() { TaskBase::cleanupHook(); }

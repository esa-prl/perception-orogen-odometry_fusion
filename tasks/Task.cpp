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

double Task::getTime(base::Time t)
{
    if (initial_time.isNull())
    {
        initial_time = t;
        return 0;
    }
    return (t - initial_time).toSeconds();
}

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
void Task::updateHook() { TaskBase::updateHook(); }

void Task::visual_delta_pose_inCallback(const base::Time& ts,
                                        const ::base::samples::RigidBodyState& delta_pose)
    {
    // Vector3d euler = delta_pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d delta_euler = OdometryFusion::quat2eul(delta_pose.orientation);
    ObservationVector z;

    // Delta euler angles are reversed to represent a local angular velocity
    // Note that when (k-1) euler angles are zero the matrices that convert
    // Time Derivative of Euler Angles ZYX <=> Angular Velocity
    // are anti-diagonal [0 0 1; 0 1 0; 1 0 0]
    z << delta_pose.position, delta_euler.reverse();
    cout << "[ODOMETRY_FUSION VISUAL " << getTime(delta_pose.time) << "]" << z.format(singleLine)
         << endl;
    ObservationVector Rd;
    Rd << 1, 1, 1, .1, .1, .1;  // TODO: make configurable
    Rd *= 0.05;
    ObservationCovarianceMatrix R = Rd.asDiagonal();
    R = R.transpose().eval() * R;
    library->update(delta_pose.time, z, R);

    outputPortPose();
}

void Task::inertial_delta_pose_inCallback(const base::Time& ts,
                                          const ::base::samples::RigidBodyState& delta_pose)
{
    InputVector u;
    if (delta_pose.velocity.hasNaN() or delta_pose.angular_velocity.hasNaN())
    {
        throw std::runtime_error("[ODOMETRY_FUSION] Invalid velocity or angular velocity");
        return;
    }
    else
    {
        u << delta_pose.velocity, delta_pose.angular_velocity;
    }
    cout << "[ODOMETRY_FUSION INERTIAL" << getTime(delta_pose.time) << "]" << u.format(singleLine)
         << endl;
    InputVector Cd;
    Cd << 1, 1, 1, .1, .1, .1;  // TODO: make configurable
    Cd *= 0.001;
    InputCovarianceMatrix C = Cd.asDiagonal();
    C = C.transpose().eval() * C;
    library->predict(delta_pose.time, u, C);

    outputPortPose();
}

void Task::outputPortPose()
{
    RigidBodyState pose_out;
    pose_out.time = library->getCurrentTime();
    StateVector x = library->getState();
    pose_out.position = x.head(3);
    pose_out.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(x(3), Eigen::Vector3d::UnitZ())
                                              * Eigen::AngleAxisd(x(4), Eigen::Vector3d::UnitY())
                                              * Eigen::AngleAxisd(x(5), Eigen::Vector3d::UnitX()));
    _pose_out.write(pose_out);
}

void Task::errorHook() { TaskBase::errorHook(); }
void Task::stopHook() { TaskBase::stopHook(); }
void Task::cleanupHook() { TaskBase::cleanupHook(); }


#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include <oro_action_server.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>

class Test : public RTT::TaskContext
{
private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;

public:
  Test(const std::string& name) : RTT::TaskContext(name), as(this, "test",
          boost::bind(&Test::goalCB, this, _1),
          boost::bind(&Test::cancelCB, this, _1), true)
  {
    goal_active = false;
  }

  ~Test()
  {
  }

  void updateHook()
  {
    if (goal_active)
    {
      activeGoal.setSucceeded();
      goal_active = false;
    }

    as.spinOnce();
  }

  void goalCB(GoalHandle gh)
  {
    RTT::Logger::log(RTT::Logger::Debug) << "Goal Handle !" << RTT::endlog();
    activeGoal = gh;
    gh.setAccepted();
    goal_active = true;
  }

  void cancelCB(GoalHandle gh)
  {
  }
private:

  actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> as;
  bool goal_active;
  GoalHandle activeGoal;
};

ORO_CREATE_COMPONENT(Test)

/*
 * FakeServo.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include "FakeServo.h"

FakeServo::FakeServo(const std::string& name) :
    RTT::TaskContext(name, PreOperational), setpoint_port("setpoint"), jointState_port(
      "jointState"), numberOfJoints_prop("numberOfJoints", "", 0)
{

  this->ports()->addPort(setpoint_port);
  this->ports()->addPort(jointState_port);

  this->addProperty(numberOfJoints_prop);
}

FakeServo::~FakeServo()
{

}

bool FakeServo::configureHook()
{
  if ((numberOfJoints = numberOfJoints_prop.get()) == 0)
    return false;
  jointState.resize(numberOfJoints);
  return true;
}

bool FakeServo::startHook()
{

  for (int i = 0; i < numberOfJoints; i++)
  {
    jointState[i].position = 0.0;
    jointState[i].velocity = 0.0;
    jointState[i].acceleration = 0.0;
  }
  return true;
}

void FakeServo::updateHook()
{
  if (setpoint_port.read(setpoint) == RTT::NewData)
  {
    if (setpoint.size() == numberOfJoints)
    {
      for (int i = 0; i < numberOfJoints; i++)
      {
        jointState[i].position = setpoint[i].position;
        jointState[i].velocity = setpoint[i].velocity;
        jointState[i].acceleration = setpoint[i].acceleration;

      }
    }
  }
  jointState_port.write(jointState);
}

ORO_CREATE_COMPONENT( FakeServo )

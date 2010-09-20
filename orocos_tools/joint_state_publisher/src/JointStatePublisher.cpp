
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include "JointStatePublisher.hpp"

JointStatePublisher::JointStatePublisher(const std::string& name) : RTT::TaskContext(name, PreOperational), msrJnt_port("msrJnt"), jointState_port("jointState"), nJoints_prop("nJoints", "number of joints"), jointNames_prop("jointNames", "names of joints")
{
	ports()->addPort(msrJnt_port);
	ports()->addPort(jointState_port);

	this->addProperty(nJoints_prop);
	this->addProperty(jointNames_prop);
}

JointStatePublisher::~JointStatePublisher()
{
}

bool JointStatePublisher::configureHook()
{
	nJoints = nJoints_prop.get();
//	std::vector<std::string> names;// = jointNames_prop.get();
	
	names.resize(nJoints);

	for(int i =0; i<nJoints; i++)
	{
		names[i] = ((RTT::Property<std::string>*)this->getProperty(std::string("joint")+(char)(i+48)+"_name"))->get();
	}
	if(nJoints != names.size())
	{
		return false;
}
	
	jState.name.resize(nJoints);
	jState.position.resize(nJoints);

	for(int i = 0; i < nJoints; i++)
	{
		jState.name[i] = names[i].c_str();
	}

	return true;
}

void JointStatePublisher::updateHook()
{
	if(msrJnt_port.read(msrJnt) == RTT::NewData)
	{
		if(msrJnt.size() == nJoints)
		{
			RTT::os::TimeService::nsecs n = RTT::os::TimeService::Instance()->getNSecs();
			jState.header.stamp.sec = n/1000000000;
			jState.header.stamp.nsec = n%1000000000;
			for(int i = 0; i < nJoints; i++)
				jState.position[i] = msrJnt[i];
			jointState_port.write(jState);
		}
	}
}

ORO_CREATE_COMPONENT( JointStatePublisher )

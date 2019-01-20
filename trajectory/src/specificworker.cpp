/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
	{ 
		robotname = params.at("RobotName").value;
	} 
	catch(const std::exception &e){ std::cout << e.what() << "SpecificWorker::SpecificWorker - Robot name defined in config. Using default 'robot' " << std::endl;}
	std::string innermodel_path;
	try
	{
		innermodel_path = params.at("NavigationAgent.InnerModelPath").value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(std::exception e) { std::cout << innermodel_path << std::endl; qFatal("Error reading InnerModel name"); }

	configparams = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	viewer = std::make_shared<InnerViewer>(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized

	// Initializing PathFinder
	pathfinder.initialize(innerModel, viewer, configparams, laser_proxy, omnirobot_proxy);

	timer.start(300);
}

void SpecificWorker::compute()
{
	try 
	{
		omnirobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		//	qDebug() << "SpecificWorker::compute" << bState.x << bState.z << bState.alpha;
	}
	catch (const Ice::Exception &ex) 
	{	std::cout << ex << std::endl; }

	pathfinder.run(); //projector is comented because the social navigation cant acceded to the laser if the astraRGBD is working

	//update viewer
	QVec robotpos = innerModel->transformS6D("world", robotname);
	viewer->ts_updateTransformValues(QString::fromStdString(robotname), robotpos);
    viewer->run();
}

///////////////////////////////////////////////////////////////////////
/////  SERVANTS
//////////////////////////////////////////////////////////////////////

float SpecificWorker::TrajectoryRobot2D_go(const TargetPose &target)
{
	if (target.doRotation)
		pathfinder.go_rot(target.x, target.z, target.ry);
	else
		pathfinder.go(target.x, target.z);
	return 0.0;
}

void SpecificWorker::TrajectoryRobot2D_mapBasedTarget(const NavigationParameterMap &parameters)
{
	//implementCODE

}

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick) 
{
	std::cout << __FUNCTION__ << " " << myPick.x << " " << myPick.z << std::endl;
	pathfinder.go(myPick.x, myPick.z);
}

NavState SpecificWorker::TrajectoryRobot2D_getState()
{
	//implementCODE
	return NavState();
}

float SpecificWorker::TrajectoryRobot2D_goBackwards(const TargetPose &target)
{
	//implementCODE
	return 0;
}

void SpecificWorker::TrajectoryRobot2D_stop()
{
	//implementCODE

}

void SpecificWorker::TrajectoryRobot2D_setHumanSpace(const PolyLineList &polyList)
{
	//implementCODE

}

float SpecificWorker::TrajectoryRobot2D_goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold)
{
	//implementCODE
	return 0;
}

float SpecificWorker::TrajectoryRobot2D_changeTarget(const TargetPose &target)
{
	//implementCODE
	return 0;
}
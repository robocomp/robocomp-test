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
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
		laser1 = innerModel->getNode<InnerModelLaser>(std::string("laser1"));
		laser2 = innerModel->getNode<InnerModelLaser>(std::string("laser2"));

	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }

	//read configuracion
	try{
		laser1Conf = laser_proxy->getLaserConfData();
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading laser1 configuration: " << e << std::endl;
	}
	try{
		laser2Conf = laser1_proxy->getLaserConfData();
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading laser2 configuration: " << e << std::endl;
	}


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//computeCODE
//QMutexLocker locker(mutex);
	//read laser1
	try
	{
		laser1Data = laser_proxy->getLaserData();
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading laser1 data" << e << std::endl;
	}
	//read laser2
	try
	{
		laser2Data = laser1_proxy->getLaserData();
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading laser2 data" << e << std::endl;
	}
	//compute laser values
	laserMix.clear();
	for (auto &&[angle, dist]: laser1Data)
	{
		addLaserPoint(laser1, dist, angle);
	}
	for (auto &&[angle, dist]: laser2Data)
	{
		addLaserPoint(laser2, dist, angle);
	}
}


void SpecificWorker::addLaserPoint(InnerModelLaser* node,const float &dist,const float &angle)
{
	QVec v1 = node->laserTo(std::string("robot"), dist, angle);
	RoboCompLaser::TData data;
	data.dist = v1.norm2();
	data.angle = atan2(v1.z(), v1.x());
	laserMix.push_back(data);
}

TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
	return laserMix;

}

LaserConfData SpecificWorker::Laser_getLaserConfData()
{
	std::cout<<"Not implemented"<<std::endl;
	return laser1Conf;
}

TLaserData SpecificWorker::Laser_getLaserData()
{
	return laserMix;
}



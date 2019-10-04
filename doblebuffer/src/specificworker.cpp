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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit computetofinalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }

	defaultMachine.start();
	return true;
}

void SpecificWorker::initialize(int period)
{
	cv::namedWindow("simplecamera",1);
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(period);
	emit this->initializetocompute();
	connect(&readtimer, &QTimer::timeout, this, &SpecificWorker::read);
	readtimer.start(20);

}

void SpecificWorker::compute()
{
	try
	{	
		//qDebug() << "compute";
		RoboCompCameraSimple::TImage img;
		camerasimple_proxy->getImage(img);
		db.put(img);
		
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}
}

void SpecificWorker::read()
{
	auto shit = db.get();
	if(shit.image.size()>0)
	{
		cv::Mat img(cv::Size(shit.height, shit.width), CV_8UC3, &shit.image[0]);
		cv::imshow("simple", img);
    	cv::waitKey(10);
	}
}


void SpecificWorker::sm_compute()
{
	//std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}







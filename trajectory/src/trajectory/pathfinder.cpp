/*
 * Copyright 2017 pbustos <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "pathfinder.h"

using namespace robocomp::pathfinder;

///////////////////////////////////////////////////////////////////
///  Public Interface
//////////////////////////////////////////////////////////////////

void PathFinder::go(float x, float z, const ParameterMap &parameters)
{
	qDebug() << "--------------------------------------------------------------------------";
	qDebug() << __FUNCTION__ << "PathFinder::go New target arrived:" << x << z;

	//Road &road = getRoad();
	controller.stopTheRobot();
	road.reset();
	road.setRequiresReplanning(true);
	currenttarget->setTranslation(QVec::vec3(x,0,z));
	//releaseRoad();
};

void PathFinder::go_rot(float x, float z, float rot, const ParameterMap &parameters)
{
	qDebug() << "--------------------------------------------------------------------------";
	qDebug() << __FUNCTION__ << "PathFinder::go_rot New target arrived:" << x << z << rot;
	//Road &road = getRoad();
	
	controller.stopTheRobot();
	
	road.reset();
	road.setRequiresReplanning(true);
	currenttarget->setTranslation(QVec::vec3(x,0,z));
	currenttarget->setRotation(QVec::vec3(0,rot,0));
	currenttarget->setHasRotation(true);
	//releaseRoad();
};


///////////////////////////////////////////////////////////////////
void PathFinder::initialize( const std::shared_ptr<InnerModel> &innerModel_,
							 const std::shared_ptr<InnerViewer> &viewer_, 
							 const shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_, 
							 LaserPrx laser_prx, OmniRobotPrx omnirobot_proxy )
{
	//shared copy of innermodel
	innerModel = innerModel_;
	viewer = viewer_;
	configparams = configparams_;
	
	//Initialize global state class 
	state = std::make_shared<NavigationState>();
	
	/// Initialize the elastic road
	road.initialize(innerModel, state, configparams);
	
	/// Initialize currentarget
	currenttarget = std::make_shared<CurrentTarget>();

	/// Initialize the Planner
	pathplanner.initialize(currenttarget, innerModel, state, configparams);
	
	/// Initialize the Projector
	projector.initialize(innerModel, currenttarget, state, configparams, laser_prx);

	/// Initialize the low level controller that drives the robot on the road	
	controller.initialize(innerModel, laser_prx, configparams, omnirobot_proxy);

	rDebug2(("PathFinder: All objects initialized"));
		
	/// Threads
	//thread_planner = std::thread(&PathPlanner::run, &pathplanner, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	
	//thread_projector = std::thread(&Projector::run, &projector, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	
	//thread_controller = std::thread(&Controller::run, &controller, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	
	std::cout << __FUNCTION__ << "PathFinder: All threads initialized" << std::endl;
	rDebug2(("PathFinder: All threads initialized"));
}

void PathFinder::run()
{
 	//while(true)
	{	
		road.update();
        projector.update(road);
		//controller.update(road);
		pathplanner.update(road);
		//TODO Revisar para pasar un shared_ptr
		drawroad.draw(road, viewer.get(), currenttarget);		
		drawroad.drawmap(pathplanner, viewer.get(), pathplanner.fmap);
		
	//	std::this_thread::sleep_for(200ms);
	}
}


// void PathFinder::innerModelChanged (const std::shared_ptr<InnerModel> &innerModel_, SNGPersonSeq persons_, SNGPolylineSeq intimate,SNGPolylineSeq personal,SNGPolylineSeq social,SNGPolylineSeq object,SNGPolylineSeq objectsblocking)
// {
// //	qDebug()<<__FUNCTION__<< "--------------ESPERANDO GET ROAD -----------------------";
// 	innerModel = innerModel_;
// 	pathplanner.persons = persons_;
// 	Road &road = getRoad();
// 		pathplanner.reloadInnerModel(innerModel_) ;
// 		pathplanner.modifyGraph(intimate, personal,social, object,objectsblocking);
// 		road.reloadInnerModel( innerModel_ ) ;  
// 		projector.reloadInnerModel(innerModel_) ;  
// 		projector.update_polyline(personal); //para el laser
// 		controller.reloadInnerModel( innerModel_ );
// 	releaseRoad();
// //	qDebug()<<__FUNCTION__<< "--------------TERMINA GET ROAD -----------------------";
// }

//////////////////////////////////////////////////
Road& PathFinder::getRoad()
{
	mymutex.lock();
	//innerModel.lock();
	
	return road;	
}

void PathFinder::releaseRoad()
{
	//innerModel.unlock();
	mymutex.unlock();
}




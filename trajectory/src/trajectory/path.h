/*
 * Copyright 2013 <copyright holder> <email>
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

#ifndef PATH_H
#define PATH_H

#include <CommonBehavior.h>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include <qmat/qline2d.h>
#include "currenttarget.h"
#include "navigationstate.h"

// class WayPoint
// {
// 	public:
// 		WayPoint()			  { pos = QVec::zeros(3); isVisible = true; hasRotation = false;};
// 		WayPoint(QVec p) 	  { pos = p; isVisible = true; hasRotation = false;};
// 		~WayPoint()			  {};
	
// 		//For ElasticBand
// 		QVec pos;																													// 3D point (x,y,z)
// 		QVec rot;																													// Euler angles (0,ry,0)
// 		float minDist, minDistAnt = std::numeric_limits<float>::max();
// 		QVec minDistPoint; 																								//In world ref system
// 		float bMinusY, bPlusY, bMinusX, bPlusX;
// 		bool minDistHasChanged;
// 		QString centerTransformName, centerMeshName, centerLineName, centerPointName, ballTransformName, ballMeshName;
// 		bool isVisible;
// 		float initialDistanceToNext;
// 		float visibleLaserAngle;
// 		float visibleLaserDist;
// 		QVec posInRobotFrame;
// 		bool hasRotation;
// }; 

class Path
{
	public:
	 	Path() = default;
		void initialize( const std::shared_ptr<InnerModel> &inner, 
						 const std::shared_ptr<NavigationState> &state_,
						 const std::shared_ptr<RoboCompCommonBehavior::ParameterList> &params);
		void readPATHFromFile(const std::shared_ptr<InnerModel> &innerModel, std::string name);
		void readPATHFromList(QList<QVec> list);
		void readPATHFromList(const std::list<QVec> &list);
		void printRobotState(const std::shared_ptr<InnerModel> &innerModel /*, const CurrentTarget& currentTarget*/);
		void print() const;
		//QList<QVec> backList;
		
		/**
		* @brief Computes all scalar values defining the interaction of the Robot and the PATH. After executing update, all variables are ready to be used 
		* through thier get methods
		* @return void
		*/
		void update();
		
	private:
		float MINIMUM_SAFETY_DISTANCE;
		float ROBOT_RADIUS;
		
		std::string robotname = "robot";
		bool active = false;;
		
		std::shared_ptr<InnerModel> innerModel;
		//Global nav state
		std::shared_ptr<NavigationState> state;

		//First point
		
		//Last point
};

#endif // PATH_H

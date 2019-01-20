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

#ifndef PROJECTOR_H
#define PROJECTOR_H
#include <genericworker.h>
#include <CommonBehavior.h>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <innermodel/innermodeldraw.h>
#include <genericworker.h>
#include <Laser.h>
#include <limits>       
#include "road.h"
#include <assert.h>
#include "currenttarget.h"
#include "linesimplifier/simplifyPath.h"
#include "intersection.h"
#include "navigationstate.h"


#define FORCE_DISTANCE_LIMIT (ROBOT_WIDTH*1.5)  //mm
#define ROBOT_STEP (ROBOT_WIDTH * 0.1)
#define DELTA_H (ROBOT_WIDTH * 0.1)
#define ROAD_STEP_SEPARATION (ROBOT_LENGTH * 0.7)

/**
 * @brief This class computes laser-road force interaction, effectively projecting the "mental" road onto the physical world of distances
 * 
 */
class Projector
{
	public:
		
		
		using InnerPtr = std::shared_ptr<InnerModel>;
		Projector() = default;
		void initialize(const InnerPtr &innerModel_, 
						const std::shared_ptr<CurrentTarget> &currenttarget_,
						const std::shared_ptr<NavigationState> &state_,
						const std::shared_ptr<RoboCompCommonBehavior::ParameterList > &configparams_, 
						LaserPrx laser_proxy_);
		void update(Road &road);
		void run(std::function<Road&()> getRoad, std::function<void()> releaseRoad);
		void reloadInnerModel(const InnerPtr &innerModel_);
		bool addPoints(Road& road);
		
		// SNGPolylineSeq polyline;
		// void update_polyline (SNGPolylineSeq polyline_){ polyline = polyline_; }
		// typedef struct { float dist; float angle;} LocalPointPol;
		//RoboCompLaser::TLaserData modifyLaser (RoboCompLaser::TLaserData laserData, SNGPolylineSeq l);
		

	private:		
		bool update(Road& road, const TLaserData& laserData, uint iter = 1);
		
		/**
		* @brief Computes de numerical derivative of the laser force field wrt to the point, by perturbing it locally.
		* @param innermodel
		* @param ball, point of the trajectory to be analyzed
		* @param laserData
		* @param forceDistanceLimit, max effective action of the force field.
		*/
		void computeDistanceField(WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit);
		
		/**
		* @brief Computes the forces exerted on the elements of the road and updates it following a simplified version on Newton physics
		*	 
		* @param innerModel ...
		* @param road ...
		* @param laserData ...
		* @return float total force exerted on the trajectory as the sum of individual changes
		*/
		float computeForces(Road &road, const RoboCompLaser::TLaserData& laserData);
		
		/**
		* @brief Removes points from the band if two of them are too close, ROBOT_RADIUS/3.
		* 
		* @param road ...
		* @return void
		*/
		bool cleanPoints(Road &road);
		
		/**
		* @brief A point of the road is visible if it is between the robot and the laser beam running through it, and if the previous point was visible
		* All points in the road are updated
		* @param road ...
		* @param laserData ...
		* @return bool
		*/
		bool checkVisiblePoints(Road &road, const RoboCompLaser::TLaserData &laserData);
		
		bool shortCut(Road& road, const RoboCompLaser::TLaserData &laserData);
		
		/**
		* @brief Check if any of the waypoints has nan coordinates
		* 
		* @param road ...
		* @return bool
		*/
		bool checkIfNAN(const Road &road);
		
		/**
		* @brief Moves a virtual copy of the robot along the road checking for enough free space around it
		* 
		* @param innermodel ...
		* @param road ...
		* @param laserData ...
		* @param robotRadius ...
		* @return bool
		*/
		bool checkCollisionAlongRoad(const RoboCompLaser::TLaserData &laserData, Road &road, Road::const_iterator robot,
		                             Road::const_iterator target, float robotRadius);

		simplifyPath simPath;

		// Points along robot's contour in robot's coordinate system
		QMat pointsMat;
		
		//Robot dimensions
		float ROBOT_LENGTH;
		float ROBOT_WIDTH;
		float ROBOT_RADIUS;
		float ATRACTION_FORCE_COEFFICIENT;
		float REPULSION_FORCE_COEFFICIENT;
		
		//Local copies
		InnerPtr innerModel;
		std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
		std::shared_ptr<CurrentTarget> currenttarget;
		
		LaserPrx laser_proxy;
		
		//Global navigation state
		const std::shared_ptr<NavigationState> state;
		
};

#endif // PROJECTOR_H

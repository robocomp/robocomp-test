/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <CommonBehavior.h>
#include "genericworker.h"
#include <OmniRobot.h>
#include "road.h"
#include <innermodel/innermodel.h>
#include <Laser.h>
#include <assert.h>


class Controller
{
	public:


		Controller() = default;
		void run(std::function<Road&()> getRoad, std::function<void()> releaseRoad);
		void initialize(const std::shared_ptr<InnerModel> &innerModel_,
				LaserPrx laser_prx, 
				std::shared_ptr<RoboCompCommonBehavior::ParameterList> params,
				OmniRobotPrx omnirobot_proxy_,
				int delay=2 /*secs*/);

		void update(Road &road);
		bool update(const std::shared_ptr<InnerModel> &innerModel, RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy, 
					Road& road, 
					bool print = false);
		void stopTheRobot();
		float angmMPI(float angle);
		void reloadInnerModel(const std::shared_ptr<InnerModel> &innerModel_);
		bool finalRotation(RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy, Road &road, bool print = false);
		
	private:
		std::shared_ptr<InnerModel> innerModel;
		OmniRobotPrx omnirobot_proxy;
		QTime time;
		int delay;
		bool avoidanceControl(const std::shared_ptr<InnerModel> &innerModel, const RoboCompLaser::TLaserData& laserData, float& vadvance, float& vrot);
		std::vector<float> baseOffsets;
		
	  // Constants reassigned to the params values
		float MAX_ADV_SPEED;
		float MAX_ROT_SPEED;
		float MAX_SIDE_SPEED;
		float MAX_LAG; //ms
		float ROBOT_RADIUS_MM; //mm
		float ARRIVAL_TOLERANCE = 20.f;  //Default tolerance on arrival

		/**
		* @brief Computes de exponential of value with parameters computed from anchor points
		* @param value quantity to be tranformed
		* @param xValue for a point with xValue in X axis
		* @param yValue we want an yValue in Y axis
		* @param min ad if the result is less than min then the result is min
		* @return float
		*/
		float exponentialFunction(float value, float xValue, float yValue, float min = 0.f);

		/**
		* @brief Computes the distance from the laser center to the external limit of robot base, along each beam angle
		* @param innerModel ...
		* @param laserData ...
		* @return std::vector< float, std::allocator >
		*/
		std::vector<float> computeRobotOffsets(const std::shared_ptr<InnerModel> &innerModel, const RoboCompLaser::TLaserData &laserData);
	
		template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0));}
};

#endif // CONTROLLER_H

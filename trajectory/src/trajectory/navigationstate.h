/*
 * Copyright 2017 <copyright holder> <email>
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

#ifndef NAVIGATIONSTATE_H
#define NAVIGATIONSTATE_H

#include <TrajectoryRobot2D.h>

class NavigationState
{
	public:
    	unsigned long elapsedTime = 0;
		std::string state = "IDLE";
		unsigned long estimatedTime = 0;
		unsigned long planningTime = 0;
		float x = 0.f;
		float z = 0.f;
		float ang = 0.f;
		float advV = 0.f;
		float rotV = 0.f;
		float distanceToTarget = 0.f;
		std::string description = "";
		
	RoboCompTrajectoryRobot2D::NavState toIce()
	{
		RoboCompTrajectoryRobot2D::NavState ns;
		
		ns.elapsedTime = elapsedTime;
		ns.state = state;
		ns.estimatedTime = estimatedTime;
		ns.planningTime = planningTime;
		ns.x = x;
		ns.z = z;
		ns.ang = ang;
		ns.advV = advV;
		ns.rotV = rotV;
		ns.distanceToTarget = distanceToTarget;
		ns.description = description;
		
		return ns;
	}
};

#endif // NAVIGATIONSTATE_H

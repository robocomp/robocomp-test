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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "trajectory/pathfinder.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	using InnerPtr = std::shared_ptr<InnerModel>;
	using InnerViewerPtr = std::shared_ptr<InnerViewer>;

	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	NavState TrajectoryRobot2D_getState();
	float TrajectoryRobot2D_goBackwards(const TargetPose &target);
	void TrajectoryRobot2D_stop();
	void TrajectoryRobot2D_setHumanSpace(const PolyLineList &polyList);
	float TrajectoryRobot2D_goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold);
	float TrajectoryRobot2D_changeTarget(const TargetPose &target);
	float TrajectoryRobot2D_go(const TargetPose &target);
	void TrajectoryRobot2D_mapBasedTarget(const NavigationParameterMap &parameters);

	void RCISMousePicker_setPick(const Pick &myPick);

public slots:
	void compute();
	void initialize(int period);

private:
	// Target info
	RoboCompTrajectoryRobot2D::TargetPose currentTarget;

	// New TrajectoryRobot Class
	robocomp::pathfinder::PathFinder pathfinder;
	std::string robotname = "robot";
	RoboCompGenericBase::TBaseState bState;
	InnerPtr innerModel;
	InnerViewerPtr viewer;
	std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
};

#endif

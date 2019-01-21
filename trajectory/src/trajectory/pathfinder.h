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

#ifndef PATHFINDER_H
#define PATHFINDER_H
#include <genericworker.h>
#include <CommonBehavior.h>
#include <thread>
#include <mutex>
#include <memory>
#include <functional>
#include "currenttarget.h"
#include "road.h"
#include "path.h"
#include "sampler.h"
#include "pathplanner.h"
#include "drawroad.h"
#include "projector.h"
#include "controller.h"
#include "navigationstate.h"
#include <qlog/qlog.h>

using namespace std;

namespace robocomp
{
	namespace pathfinder
	{
		typedef map<string, string> ParameterMap;

		class PathFinder
		{
			private:
				#ifdef USE_QTGUI
					using InnerViewerPtr = std::shared_ptr<InnerViewer>;
					InnerViewerPtr viewer;
				#endif
				using InnerPtr = std::shared_ptr<InnerModel>;
				InnerPtr innerModel;
				std::shared_ptr<NavigationState> state;
				mutable std::mutex mymutex;

				std::shared_ptr<CurrentTarget> currenttarget;
				std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
				
				DrawRoad drawroad;
				Projector projector;
				Controller controller;
				Road road;
				PathPlanner pathplanner;


			public:
                PathFinder() = default;
				void initialize(const std::shared_ptr<InnerModel> &innerModel_,
								const std::shared_ptr<InnerViewer> &viewer_, 
								const shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_,
								LaserPrx laser_prx,
								OmniRobotPrx omnirobot_proxy);
				void releaseRoad();
				Road& getRoad();
				RoboCompTrajectoryRobot2D::NavState getState(){ return state->toIce(); };
				
				/////////////////////////////
				/// Interface
				////////////////////////////
				void go(float x, float z, const ParameterMap &parameters = ParameterMap());
				void go_rot(float x, float z, float rot, const ParameterMap &parameters = ParameterMap());
                // vector <int32_t> getHumanBlocking(){return pathplanner.pId_blocking;};
                // vector <int32_t> getHumanSoftBlocking(){return pathplanner.pId_softblocking;};
                // vector <int32_t> getAffordanceBlocking(){return pathplanner.pId_affblocking;};


				//void setInnerModel(InnerModel* innerModel_){ innerModel = innerModel_; };
				//void innerModelChanged(const std::shared_ptr<InnerModel> &innerModel_, SNGPersonSeq persons, SNGPolylineSeq intimate,SNGPolylineSeq personal,SNGPolylineSeq social, SNGPolylineSeq objects,SNGPolylineSeq objectsblocking);
						
				void run();
				///////////////////////////
		};
	} //path
} //robocomp

#endif // PATHFINDER_H

/*
 * Copyright 2016 pbustos <email>
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

#ifndef DRAWROAD_H
#define DRAWROAD_H

#include "innerviewer.h"
#include "road.h"
#include "pathplanner.h"
#include <qmat/qline2d.h>
#include <innermodel/innermodeldraw.h>


class DrawRoad
{
	public:
		/**
		 * @brief Draws the road on an InnerModelViewer
		 * 
		 * @param road to be pictured
		 * @param viewer InnerModelViewer where to do de painting
		 * @param currentTarget structure containing current target data
		 * @return bool
		 */
		void draw(Road& road, InnerViewer* viewer, std::shared_ptr<CurrentTarget> currenttarget);
		void drawmap(PathPlanner &pathplanner, InnerViewer *viewer, PathPlanner::FMap const& fmap );
};

#endif // WAYPOINTSDRAW_H

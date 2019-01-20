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

#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <genericworker.h>
#include <iostream>
#include <thread>
#include <memory>
#include <chrono>
// #include <innermodel/innermodelmgr.h>
#include "road.h"
#include "currenttarget.h"
#include "sampler.h"
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <qlog/qlog.h>
#include "navigationstate.h"
#include "safepolylist.h"
#include <QPolygon>
#include <QPoint>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;





#define TILE_SIZE 300 // grid discrtization step
 
template<class T> auto operator<<(std::ostream& os, const T& t) -> decltype(t.print(os), os) 
{ 
    t.print(os); 
    return os; 
};

class Log
{
 	public:
 		Log()
		{  prev << "Logger: " <<  __TIME__ << " "; };
		std::ostringstream prev;
		void print(){ std::cout << prev.str() << std::endl;}
};
inline Log operator<<(Log log, const std::string &text){ log.prev << __FUNCTION__ << text; log.print(); return log; };

class
PathPlanner
{
		typedef std::chrono::high_resolution_clock clock;
		struct Key
		{
			long int x;
			long int z;
		
			public:
				Key(): x(0), z(0) {};
				Key(long int &&x, long int &&z): x(std::move(x)), z(std::move(z)){};
				Key(long int &x, long int &z): x(x), z(z){};
				Key(const long int &x, const long int &z): x(x), z(z){};
				bool operator==(const Key &other) const
					{ return x == other.x && z == other.z; };
				void print(std::ostream &os) const 	{ os << " x:" << x << " z:" << z; };
		};
			
		struct Value
		{
			uint id;
			bool free;
			float cost;
		};

		struct KeyHasher
			{
				std::size_t operator()(const Key& k) const
				{
					using boost::hash_value;
					using boost::hash_combine;

					// Start with a hash value of 0    .
					std::size_t seed = 0;

					// Modify 'seed' by XORing and bit-shifting in one member of 'Key' after the other:
					hash_combine(seed,hash_value(k.x));
					hash_combine(seed,hash_value(k.z));
					return seed;
				};
			};	
			
	public:
		using InnerPtr = std::shared_ptr<InnerModel>;
		PathPlanner() = default;
		~PathPlanner() = default;
		void initialize(const std::shared_ptr<CurrentTarget> &currentarget_,
						const InnerPtr &innerModel, 
						const std::shared_ptr<NavigationState> &state_,
						const std::shared_ptr<RoboCompCommonBehavior::ParameterList> &configparams);
		void run(std::function<Road&()> handler, std::function<void()> releaseRoad);
		void update(Road &road);
		bool get_map_dirty_bit() const 		{return map_dirty_bit;};  
		void set_map_dirty_bit(bool v=true)	{map_dirty_bit = v;};
		void reloadInnerModel(const InnerPtr &innerModel_);


		typedef	std::unordered_map<PathPlanner::Key, PathPlanner::Value, PathPlanner::KeyHasher> FMap;	
		FMap fmap;
		FMap fmap_initial;
		
		///////////////////////////////////////
		struct Point { float x; float z;};
		Point point;
		vector <Point> initialp_list;
		vector <Point> occupied_list;
		vector <Point> containedp_list;
		//////////////////////////////////////
		// void modifyGraph(SNGPolylineSeq intimate,SNGPolylineSeq personal, SNGPolylineSeq social, SNGPolylineSeq object,SNGPolylineSeq objectsblocking);
		// void modifyCost(SNGPolylineSeq personal, SNGPolylineSeq social, SNGPolylineSeq object);
		// SNGPolylineSeq polylines_block;
		// SNGPolylineSeq polylines_softblock;
        // SNGPolylineSeq polylines_aff;
        // SNGPersonSeq persons;

        // void checkHumanBlock(Road &road);
        // vector <int32_t> pId_blocking = {};
        // bool checkHumanSoftBlock(std::list<QVec> currentPath);
        // vector <int32_t> pId_softblocking = {};
        // bool checkAffordances(std::list<QVec> currentPath);
        // vector <int32_t> pId_affblocking = {};
	/////////////////////////////////////////////////////////
	private:
		std::shared_ptr<CurrentTarget> currenttarget;
		InnerPtr innerModel;
		std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
		std::string robotname = "robot";
		bool map_dirty_bit = true;
		
		//Sampler of free space
		Sampler sampler;		
		
		//Global state of the navigatin process to be returned to clients
		std::shared_ptr<NavigationState> state;
		
		/**
			* @brief Main callable method. Computes a path for the robot given a target pose and a point to Innermodel. inner is used to synchronize the           current copy of Innermodel.
			* This is done so the planner can ran as a thread in the future.
			* Should de extended to return the best path found given a certain time limit
			* @param target CurrentTarget type showing the current goal assigned to the PathFinder
			* @param inner Pointer to Innermodel
			* @return true if a path of at least two waypoints have been found. Path is written to class variable currentPath.
			* Additional chekcs on max and min lenght for the path should be applied
		*/
		std::list<QVec> computePath(const Road &road, std::shared_ptr<CurrentTarget> target);
		
		/**
		 * @brief Constructs a regular grid graph marking nodes as free/occupied, after checking free space with the sampler.
		 * We need the complete space bounding box and a step size
		 * 
		 */
		void constructGraph(FMap &fmap, unsigned tile_size);

		Key pointToGrid(const QVec &p);
		std::vector<std::pair<PathPlanner::Key,PathPlanner::Value> > neighboors(const Key &k) const;
		std::list<QVec> djikstra(const FMap &graph, Key const& source, Key const& target);
		std::list<QVec> orderPath(const std::vector<std::pair<uint,Key>> &previous, const Key &source, const Key &target);
		QRectF outerRegion;
		int hmin, hmax, vmin, vmax;
		
};



#endif // PATHPLANNER_H

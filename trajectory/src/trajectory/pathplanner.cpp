#include "pathplanner.h"

using namespace std::chrono_literals;

void PathPlanner::initialize(const std::shared_ptr<CurrentTarget> &currenttarget_, 
							 const InnerPtr &innerModel_, 
							 const std::shared_ptr<NavigationState> &state_, 
							 const std::shared_ptr<RoboCompCommonBehavior::ParameterList> &configparams_)
{
 	currenttarget = currenttarget_;
	innerModel = innerModel_;
	state = state_;
	configparams = configparams_;
	
	/// Initialize sampler of free space
	try
	{	sampler.initialize(innerModel, configparams); } 
	catch(const std::exception &ex) { throw ex;	}
	
	try{ robotname = configparams->at("RobotName").value;} 
	catch(const std::exception &e){ std::cout << e.what() << " PathPlanner::initialize - No Robot name defined in config. Using default 'robot' " << std::endl;}

	//Get space bounding box from config file;
	outerRegion = sampler.getOuterRegion();
	std::ostringstream a; a << "horizontal limits " << outerRegion.left() << " " << outerRegion.right();
	rDebug2((a.str()));
	
	hmin = std::min(outerRegion.left(), outerRegion.right());
	hmax = std::max(outerRegion.left(), outerRegion.right());
	vmin = std::min(outerRegion.top(), outerRegion.bottom());
	vmax = std::max(outerRegion.top(), outerRegion.bottom());
	
	rDebug2(("PathPlanner building graph......"));
	auto start = clock::now();
	sampler.lock();
		constructGraph(fmap, TILE_SIZE);
	sampler.unlock();
	auto end = clock::now();
  	std::cout << __FILE__ << __FUNCTION__ << " Graph building time " << duration_cast<milliseconds>(end-start).count() << "ms\n";
	rDebug2(("PathPlanner graph finished!"));
	
	rDebug2(("PathPlanner finished initializing"));
	Log() << __FILE__ << __FUNCTION__ << "-------------------------------------------------";
}

void PathPlanner::update(Road &road)
{
	if( road.getRequiresReplanning() )
	{
		state->state = "PLANNING";
		std::list<QVec> currentPath = computePath(road, currenttarget);
		qDebug() << __FILE__ << __FUNCTION__ << " CurrentPath length:" << currentPath.size();

			if(currentPath.empty() == false)
            {
                //pId_blocking.clear();
                //if (!checkHumanSoftBlock(currentPath) and !checkAffordances(currentPath))
                {
                    road.readRoadFromList(currentPath);
                    if (currenttarget->hasRotation())
                    {
                        road.last().rot = currenttarget->getRotation();
                        road.last().hasRotation = true;
                    }
                }
            }
			else
			{
				//pId_softblocking.clear();
				//pId_affblocking.clear();
				//std::cout << __FILE__ << __FUNCTION__ << " No path found, checking if there are humans" << std::endl;
				//checkHumanBlock(road);
			}
			road.setRequiresReplanning(false);
            road.setFinished(true);
	}
}

void PathPlanner::run(std::function<Road&()> getRoad, std::function<void()> releaseRoad)
{
	rDebug2(("PathPlanner building graph......"));
	auto start = clock::now();
	sampler.lock();
		constructGraph(fmap, TILE_SIZE);
	sampler.unlock();
	auto end = clock::now();
  	std::cout << __FILE__ << __FUNCTION__ << " Graph building time " << duration_cast<milliseconds>(end-start).count() << "ms\n";
	rDebug2(("PathPlanner graph finished!"));
		
	while(true)
	{
		Road &road = getRoad();
		if( road.getRequiresReplanning() )
		{
			state->state = "PLANNING";
			releaseRoad();
			std::list<QVec> currentPath = computePath(road, currenttarget);
			qDebug() << __FILE__ << __FUNCTION__ << " CurrentPath length:" << currentPath.size();

			Road &road = getRoad();
			if(currentPath.empty() == false)
            {
                //pId_blocking.clear();
                //if (!checkHumanSoftBlock(currentPath) and !checkAffordances(currentPath))
                {
                    road.readRoadFromList(currentPath);
                }
            }
			else
			{
				//pId_softblocking.clear();
				//pId_affblocking.clear();
				std::cout << __FILE__ << __FUNCTION__ << " No path found, checking if there are humans" << std::endl;
				//checkHumanBlock(road);
			}
			road.setRequiresReplanning(false);
            road.setFinished(true);
			releaseRoad();
		}
		else
			releaseRoad();
		std::this_thread::sleep_for(1s);
	}
}

void PathPlanner::reloadInnerModel(const InnerPtr &innerModel_)
{
	sampler.lock();
		sampler.reloadInnerModel(innerModel_);	
	sampler.unlock();
	
// 	innerModel.reset(innerModel_.get());
	innerModel = innerModel_;
}

// bool PathPlanner::checkAffordances(std::list<QVec> currentPath) //devuelve true si hay alguna persona con softblock
// {
//     qDebug()<<__FUNCTION__;
//     pId_affblocking.clear();
//     vector<QPolygonF> qp_list;

//     for (auto poly : polylines_aff)
//     {
//         QPolygonF qp;
//         for (auto p:poly)
//             qp << QPointF(p.x * 1000, p.z * 1000);

//         for (auto p:currentPath)
//         {
//             if (qp.containsPoint(QPointF(p.x(), p.z()), Qt::OddEvenFill))
//             {
//                 qp_list.push_back(qp);
//                 break;
//             }
//         }
//     }

//     for (auto polygon : qp_list)
//     {
//         for (auto p:persons)
//         {
//             if (polygon.containsPoint(QPointF(p.x* 1000, p.z* 1000), Qt::OddEvenFill))
//             {
//                 qDebug () <<"La persona situada en " <<p.x*1000 << " "<< p.z*1000 << "bloquea al robot al estar en una AFFORDANCE. CON ID" << p.id ;
//                 pId_affblocking.push_back(p.id);
//             }
//         }
//     }

//     qDebug()<<"NUMERO DE PERSONAS EN AFFORDANCE = "<<pId_affblocking.size();
//     if (pId_affblocking.size() > 0)
//         return true;
//     else
//         return false;
// }



// bool PathPlanner::checkHumanSoftBlock(std::list<QVec> currentPath) //devuelve true si hay alguna persona con softblock
// {
//     pId_softblocking.clear();
//     vector<QPolygonF> qp_list;

//     for (auto poly : polylines_softblock)
//     {
//         QPolygonF qp;
//         for (auto p:poly)
//             qp << QPointF(p.x * 1000, p.z * 1000);

//         for (auto p:currentPath)
//         {
//             if (qp.containsPoint(QPointF(p.x(), p.z()), Qt::OddEvenFill))
//             {
//                 qp_list.push_back(qp);
//                 break;
//             }
//         }
//     }

//     for (auto polygon : qp_list)
//     {
//         for (auto p:persons)
//         {
//             if (polygon.containsPoint(QPointF(p.x* 1000, p.z* 1000), Qt::OddEvenFill))
//             {
//                 qDebug () <<"La persona situada en " <<p.x*1000 << " "<< p.z*1000 << "bloquea SUAVEMEEEEEENTE el camino. CON ID" << p.id ;
//                 pId_softblocking.push_back(p.id);
//             }
//         }
//     }

//     qDebug()<<"NUMERO DE PERSONAS BLOQUEANDO SUAVEMENTE = "<<pId_softblocking.size();
//     if (pId_softblocking.size() > 0)
//         return true;
//     else
//         return false;
// }





// void PathPlanner::checkHumanBlock(Road &road)
// {
//     qDebug()<<__FUNCTION__;
// 	FMap fmap_aux = fmap;
// 	fmap = fmap_initial;
//     pId_blocking.clear();

// 	std::list<QVec> Path = computePath(road, currenttarget);

// 	if(!Path.empty())
//     {
// 		qDebug()<<"HUMANOS BLOQUEANDO EL CAMINO";
//         QPolygonF qp;

// 		for (auto poly : polylines_block)
// 		{
// 			qp.clear();
// 			for (auto p:poly)
// 				qp << QPointF(p.x * 1000, p.z * 1000);

// 			for (FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
// 			{
// 				if (qp.containsPoint(QPointF(iter->first.x, iter->first.z), Qt::OddEvenFill))
// 				{
// 					point.x = iter->first.x;
// 					point.z = iter->first.z;
// 					occupied_list.push_back(point);

// 					iter->second.free = false;
// 				}
// 			}

// 			Path = computePath(road, currenttarget);

// 			if (Path.empty())
// 				break;
// 		}

// 		qDebug()<<"Hay "<< persons.size()<<" persona en el mundo, comprobando cual bloquea al robot";
// 	for (auto p:persons)
//         {
//             if (qp.containsPoint(QPointF(p.x* 1000, p.z* 1000), Qt::OddEvenFill))
//             {
//                 qDebug () <<"La persona situada en " <<p.x*1000 << " "<< p.z*1000 << "bloquea el camino. CON ID" << p.id ;
//                 pId_blocking.push_back(p.id);
//             }
//         }

//         qDebug()<<"HAY "<< pId_blocking.size()<< " PERSONAS BLOQUEANDO EL CAMINO";

// 	}

// 	fmap = fmap_aux;
// }


///////////////////////////////////////////////////////////////////////////////////////////////////

std::list<QVec> PathPlanner::computePath(const Road &road, std::shared_ptr<CurrentTarget> target)
{
	std::cout << "PathPlanner::compute path Planning for target:" << std::endl;
	target->getTranslation().print("tr"); 
	target->getRotation().print("rot"); 
	
 	QVec robot = innerModel->transformS("world",robotname);
// 	QVec robotRotation = innerModel->getRotationMatrixTo("world", "robot").extractAnglesR_min();
// 	
	Log() << "Starting planning with robot at:";
//   	qDebug() << __FUNCTION__ << "Starting planning with robot at:" << robot <<  "and target at:" << target->getTranslation();
	
// 
// 	/////////////////////////////////////////////
// 	//If robot on obstacle we can¡t proceed
// 	/////////////////////////////////////////////
// 	if( std::get<bool>(sampler->checkRobotValidStateAtTarget(robot, robotRotation)) == false)
// 	{
// 		qDebug() << __FUNCTION__ << "Robot collides in origin. Aborting planner";  
// 		return false;
// 	}
// 	
// 	/////////////////////////////////////////////
// 	//If target on obstacle find a point close to it on the robot-target line. For now returns
// 	/////////////////////////////////////////////
// 	if( sampler->searchRobotValidStateCloseToTarget(target) == false )
// 	{
// 		qDebug() << __FUNCTION__ << "Robot collides in target. Aborting planner";  //Should search a next obs-free target
// 		return false;
// 	}
// 	
 	////////////////////////////////////////////
 	// PLanner uses another instance of InnerModel to plan so we resynchronize both before initiate planning
 	////////////////////////////////////////////
 	//innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), robotRotation.x(), robotRotation.y(), robotRotation.z());
 
 	//Create new fresh empty path;
	std::list<QVec> currentPath;   	
 	
 	/////////////////////////////////////////////
 	//Check if the target is in "plain sight"
 	/////////////////////////////////////////////
  	if ( sampler.checkRobotValidDirectionToTargetOneShot( robot, target->getTranslation()) )
  	{
		rDebug2(("Target on sight. Proceeding"));
   		currentPath.push_back(robot);
		currentPath.push_back(target->getTranslation());
		return currentPath;

  	}
 	else 
		rDebug2(("Target NOT on sight. Proceeding with grid search"));

// 	/////////////////////////////////////////////
// 	// Get closest point to robot
// 	/////////////////////////////////////////////
	
	Key robot_key = pointToGrid(robot);
	Key target_key = pointToGrid(target->getTranslation());
	std::ostringstream a; a << robot_key << " " << target_key;
	rDebug2((a.str()));
	
	return djikstra(fmap, robot_key, target_key );
	
// 	/////////////////////////////////////////////
// 	//If not, search in KD-tree for closest points to origin and target
// 	/////////////////////////////////////////////
// 	Vertex robotVertex, targetVertex;
// 	searchClosestPoints(robot, target, robotVertex, targetVertex);
// 	qDebug() << __FUNCTION__ << "Computing closest point to robot in graph";
// 	qDebug() << __FUNCTION__ << "Closest point to robot:" << graph[robotVertex].pose;
// 	qDebug() << __FUNCTION__ << "Closest point to targett:" << graph[targetVertex].pose;
// 	
// 	/////////////////////////////////////////////
// 	// Check if the closest point in the graph is in "plain sight"
// 	// Otherwise call the planner
// 	/////////////////////////////////////////////
// 	if ( sampler->checkRobotValidDirectionToTargetOneShot( robot, graph[robotVertex].pose) )
// 	{
//  		qDebug() << __FUNCTION__ << "Closest point to robot in graph is ON sight";
// 		currentPath << robot << graph[robotVertex].pose;
// 	}
// 	else
// 	{
// 		qDebug() << __FUNCTION__ << "Closest point to robot in graph is NOT on sight. Proceeding to search";
// 		/////////////////////////////////////////////
// 		//Obtain a free path from [robot] to [robotVertex] using RRTConnect. Return if fail.
// 		/////////////////////////////////////////////
// 		qDebug() << __FUNCTION__ << "Searching with RRT from ROBOT to closest point in graph (CPG)";
// 		QList<QVec> path;
// 		if (planWithRRT(robot, graph[robotVertex].pose, path) )
// 		{
// 			path.removeLast();
// 			currentPath << path;
// 			qDebug() << __FUNCTION__ << "RRTConnect succeeded finding a path from ROBOT to CPG with a length of " << currentPath.size() << "steps.";
// 		}
// 		else
// 		{
// 			qDebug() << __FUNCTION__ << "ERROR: Path from ROBOT to CPG NOT FOUND:";
// 			return false;
// 		}
// 	}
// 
// 	/////////////////////////////////////////////
// 	//Now we are in the graph
// 	//Search in graph a minimun path using Dijkstra algorithm
// 	/////////////////////////////////////////////
// 	if( robotVertex != targetVertex )  //Same node for both. We should skip searchGraph
// 	{
// 		std::vector<Vertex> vertexPath;
// 		if ( searchGraph(robotVertex, targetVertex, vertexPath) == true)
// 			for( Vertex v : vertexPath )
// 				currentPath.append(graph[v].pose);
// 		else //No path found. The path does not connect to target's closest point in graph
// 		{
// 			qDebug() << __FUNCTION__ << "No path through graph. Starting RRTConnect from " << graph[robotVertex].pose << " to" <<  target;
// 		  QList<QVec> path;	
// 		  if ( planWithRRT(graph[robotVertex].pose, target, path) )
// 			{
// 				path.removeFirst();
// 				currentPath += path;
// 	  		qDebug() << __FUNCTION__ << "RRTConnect succeeded finding a path from CPG to CPT with a length of " << currentPath.size() << "steps.";
// 				qDebug() << __FUNCTION__ << "RRTConnect path form CPG to CPT:" <<  path;
// 			}
// 			else
// 				return false;
// 		}
// 	}
// 	else	//add the only node
// 		currentPath += graph[robotVertex].pose;
// 
// 	
// 	///////////////////////////////////////////////////////////
// 	// Check if the closest point in the graph to the TARGET is in "plain sight".
// 	// Otherwise call the planner
// 	////////////////////////////////////////////////////////////7
// 	if ( sampler->checkRobotValidDirectionToTargetOneShot( graph[targetVertex].pose, target ))
// 	{
//  		qDebug() << __FUNCTION__ << "Closest point to target in graph ON sight";
// 		currentPath << target << graph[targetVertex].pose;
// 	}
// 	else
// 	{
// 		qDebug() << __FUNCTION__ << "Closest point to target in graph NOT on sight. Proceeding to search";
// 		
// 		/////////////////////////////////////////////
// 		//Obtain a free path from target to targetVertex using RRTConnect.
// 		/////////////////////////////////////////////
// 		qDebug() << __FUNCTION__ << "Searching with RRT from closest point in graph (CPG) to TARGET";
// 		QList<QVec> path;
// 		if (planWithRRT(graph[robotVertex].pose, target, path) == true)
// 		{
// 				path.removeLast();
// 				currentPath << path;
// 		}
// 		else
// 		{
// 				qDebug() << __FUNCTION__ << "ERROR: Path from TARGET to CPG NOT FOUND using RRT planner. Can't do better";
// 				return false;
// 		}
// 	}
// 
// 	/////////////////////////////////////
// 	// Smoothing
// 	/////////////////////////////////////
// 	pathSmoother(currentPath);
// 	return true;
}

PathPlanner::Key PathPlanner::pointToGrid(const QVec &p)
{
	int kx = (p.x()-hmin)/TILE_SIZE;
	int kz = (p.z()-vmin)/TILE_SIZE;
	
	return Key(hmin + kx*TILE_SIZE, vmin + kz*TILE_SIZE);
}

/**
 * @brief Constructs the graph from current sampler
 * 
 */
void PathPlanner::constructGraph(FMap &fmap, uint tile_size)
{
	qDebug()<<__FUNCTION__;
	
	uint k=0;
 	for( long int i = hmin ; i < hmax ; i += tile_size)
 		for( long int j = vmin ; j < vmax ; j += tile_size)
 		{
			bool free = sampler.checkRobotValidStateAtTargetFast(QVec::vec3(i,10,j),QVec::zeros(3));
			fmap.emplace( Key(i,j),Value{k++,free,1}); 
 		}
 		
// 	for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter){
// 		std::cout << iter->first << " " << "COSTE " << iter->second.cost << std::endl;
// 	}
	
	static bool first = true;

	if (first)
	{
		for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
		{
			
			if (iter->second.free == false)
			{
				point.x = iter->first.x;
				point.z = iter->first.z;
				initialp_list.push_back(point);
			}
		}
		
		fmap_initial = fmap;
		first = false;
	}

	set_map_dirty_bit(true);
		
	qDebug() << __FILE__ << __FUNCTION__ << "Map size: " << fmap.size();	
}


// void PathPlanner::modifyCost(SNGPolylineSeq personal, SNGPolylineSeq social, SNGPolylineSeq object)
// {

// //	qDebug()<<__FUNCTION__;
// 	containedp_list.clear();

//     for (auto poly : object)
//     {
//         QPolygonF qp_object;

//         for (auto p: poly)
//             qp_object << QPointF(p.x*1000,p.z*1000);

//         for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
//         {
//             if (qp_object.containsPoint(QPointF(iter->first.x,iter->first.z),Qt::OddEvenFill))
//             {
//                 point.x = iter->first.x;
//                 point.z = iter->first.z;
//                 containedp_list.push_back(point);
//                 iter->second.cost = 1.5;
//             }
//         }

//     }

// 	for (auto poly : social)
// 	{
// 		QPolygonF qp_social;
		
// 		for (auto p: poly)
// 			qp_social << QPointF(p.x*1000,p.z*1000);
		
// 		for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
// 		{
// 			if (qp_social.containsPoint(QPointF(iter->first.x,iter->first.z),Qt::OddEvenFill))
// 			{
// 				point.x = iter->first.x;
// 				point.z = iter->first.z;
// 				containedp_list.push_back(point);
// 				iter->second.cost = 2.0;
// 			}
// 		}
		
// 	}
	
// 	for (auto poly : personal)
// 	{
// 		QPolygonF qp_personal;
		
// 		for (auto p: poly)
// 			qp_personal << QPointF(p.x*1000,p.z*1000);
		
			
// 		for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
// 		{
// 			if (qp_personal.containsPoint(QPointF(iter->first.x,iter->first.z),Qt::OddEvenFill))
// 			{	
// 				point.x = iter->first.x;
// 				point.z = iter->first.z;
// 				containedp_list.push_back(point);
// 				iter->second.cost = 4.0;
// 			} 
// 		}
		
// 	}
	
	
// 	for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
// 	{
// 		bool modified = false;
// 		for (auto p : containedp_list)
// 		{
// 			if (p.x == iter->first.x and p.z == iter->first.z)
// 				modified = true;
// 		}
// 		if (!modified)
// 			iter->second.cost = 1.0;
// 	}

// }

// void PathPlanner::modifyGraph(SNGPolylineSeq intimate, SNGPolylineSeq personal, SNGPolylineSeq social, SNGPolylineSeq object, SNGPolylineSeq objectsblocking)
// {

// 	qDebug()<<__FUNCTION__;
// 	polylines_block = intimate;
// 	polylines_softblock = personal;
//     polylines_aff = objectsblocking;

// 	occupied_list.clear();
	
// 	for (auto poly : intimate)
// 	{
// 		QPolygonF qp;					
// 		for (auto p:poly)
// 		{
// 			qp << QPointF(p.x*1000,p.z*1000);
// 		}	
		
// 		for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
// 		{
// 			if (qp.containsPoint(QPointF(iter->first.x,iter->first.z),Qt::OddEvenFill))
// 			{
// 				point.x = iter->first.x;
// 				point.z = iter->first.z;
// 				occupied_list.push_back(point);
				
// 				iter->second.free = false;
// 			}
// 		} 	
// 	}	

// 	for(FMap::iterator iter = fmap.begin(); iter != fmap.end(); ++iter)
// 	{
// 		point.x = iter->first.x;
// 		point.z = iter->first.z;
		
// 		bool in_initial = false;
// 		bool in_occupied = false;
		
// 		for (auto p : initialp_list)
// 		{
// 			if (p.x == point.x and p.z == point.z) in_initial = true;
// 		}
		
// 		for (auto p : occupied_list)
// 		{
// 			if (p.x == point.x and p.z == point.z) in_occupied = true;
// 		}
		
// 		  if (!in_initial and !in_occupied) iter->second.free = true;	
// 	}
	
// 	modifyCost(personal,social, object);
	
// 	set_map_dirty_bit(true);
	
	
// }


std::vector<std::pair<PathPlanner::Key,PathPlanner::Value>> PathPlanner::neighboors(const Key &k) const
{
	std::vector<std::pair<Key,Value>> neigh;
	// list of increments to access the neighboors of a given position
	const int T = TILE_SIZE;
	const std::vector<int> xincs = {T,T,T,0,-T,-T,-T,0};
	const std::vector<int> zincs = {T,0,-T,-T,-T,0,T,T};

	for (auto itx = xincs.begin(), itz = zincs.begin(); itx != xincs.end(); ++itx, ++itz)
	{
		Key lk{k.x + *itx, k.z + *itz}; 
		FMap::const_iterator it = fmap.find(lk);
		if( it != fmap.end() and it->second.free )
		{
			Value v(it->second);					// bacause iterator is const
			if (abs(*itx)>0 and abs(*itz)>0) v.cost = v.cost * 1.41;		// if neighboor in diagonal, cost is sqrt(2)
			neigh.push_back(std::make_pair(lk,v));
		}
	};
	return neigh;
}
     
std::list<QVec> PathPlanner::djikstra(const FMap &graph, const Key &source, const Key &target)
{
    std::vector<uint> min_distance(graph.size(), INT_MAX);
	std::vector<std::pair<uint,Key>> previous(graph.size(), std::make_pair(-1, Key()));
	
    min_distance[ fmap[source].id ] = 0;
	auto comp = [this](std::pair<uint,Key> x, std::pair<uint,Key> y)
		{ return x.first < y.first or (!(y.first < x.first) and this->fmap[x.second].id < this->fmap[y.second].id); };
    std::set< std::pair<uint,Key>, decltype(comp)> active_vertices(comp);
	
    active_vertices.insert({0,source});
    while (!active_vertices.empty()) 
	{
        Key where = active_vertices.begin()->second;
	
	    if (where == target) 
		{
			qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap[where].id];  //exit point 
			return orderPath(previous, source, target);
		}
        active_vertices.erase( active_vertices.begin() );
	    for (auto ed : neighboors(where)) 
		{
			//qDebug() << __FILE__ << __FUNCTION__ << "antes del if" << ed.first.x << ed.first.z << ed.second.id << fmap[where].id << min_distance[ed.second.id] << min_distance[fmap[where].id];
            if (min_distance[ed.second.id] > min_distance[fmap[where].id] + ed.second.cost) 
			{
				active_vertices.erase( { min_distance[ed.second.id], ed.first } );
                min_distance[ed.second.id] = min_distance[fmap[where].id] + ed.second.cost;
				previous[ed.second.id] = std::make_pair(fmap[where].id, where);
                active_vertices.insert( { min_distance[ed.second.id], ed.first } );
            }
		}
    }
    return std::list<QVec>();
}

/**
 * @brief Recovers the optimal path from the list of previous nodes
 * 
 * @param previous p_previous:...
 * @param source p_source:...
 * @param target p_target:...
 * @return std::__cxx11::list< RMat::QVec, std::allocator< RMat::QVec > >
 */
std::list<QVec> PathPlanner::orderPath(const std::vector<std::pair<uint,Key>> &previous, const Key &source, const Key &target)
{
	std::list<QVec> res;
	Key k = target;
	uint u = fmap[k].id;
	while(previous[u].first != (uint)-1)
	{
		QVec p = QVec::vec3(k.x, 0, k.z);
		res.push_front(p);
		u = previous[u].first;
		k = previous[u].second;
	}
	qDebug() << __FILE__ << __FUNCTION__ << "Path length:" << res.size();  //exit point 
	return res;
};


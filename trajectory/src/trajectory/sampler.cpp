/*
 * Copyright 2014 <copyright holder> <email>
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

#include "sampler.h"


/**
 * @brief Initializes the Sampler with the limits of robot's workspace and a point to innermodel.
 * The method uses that pointer to create a copy of innermodel, so the Sampler can use it to test valid 
 * robot configurations without interfering with the original one
 * 
 * @param inner pointer to innerModel object
 * @param outerRegion_ QRectF delimiting the robot's workspace
 * @param innerRegions_ List of QRectF polygons delimiting forbidden regions inside robot's workspace
 * @return void
 */
void Sampler::initialize(const std::shared_ptr<InnerModel> &inner, std::shared_ptr<RoboCompCommonBehavior::ParameterList> params_)
{
	qDebug() << "Sampler::" << __FUNCTION__;
	innerModelSampler = inner;    
	
	/// Processing configuration parameters
	try{ robotname = params_->at("RobotName").value;} catch(const std::exception &e){ std::cout << e.what() << " Sampler::initialize - No Robot name defined in config. Using default 'robot' " << std::endl;}

	try
	{	
		outerRegion.setLeft(std::stof(params_->at("OuterRegionLeft").value));
		outerRegion.setRight(std::stof(params_->at("OuterRegionRight").value));
		outerRegion.setBottom(std::stof(params_->at("OuterRegionBottom").value));
		outerRegion.setTop(std::stof(params_->at("OuterRegionTop").value));
	}
	catch(const std::exception &e)
	{ 
		std::cout << "Exception " << e.what() << " Sampler::initialize(). OuterRegion parameters not found in config file" << std::endl; 
		//robocomp::exception ex("OuterRegion parameters not found in config file");
		throw e;
	}
	
// 	innerRegions = innerRegions_;
// 	foreach(QRectF ir,  innerRegions_)
// 		if( ir.isNull() == false)
// 		{
// 			robocomp::exception ex("Sampler::initialize() An InnerRegion is not a valid rectangle");
// 			throw ex;
// 		}

	if(outerRegion.isNull())
	{
		robocomp::exception ex("Sampler::initialize() OuterRegion is not properly initialized");
		throw ex;
	}
		
	QStringList ls = QString::fromStdString(params_->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
	qDebug() << __FILE__ << __FUNCTION__ << ls.size() << "objects read for exclusion list";
	
	foreach(const QString &s, ls)
		excludedNodes.insert(s);
	
	// Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
	robotNodes.clear(); restNodes.clear(); 
	recursiveIncludeMeshes(innerModelSampler->getRoot(), "robot", false, robotNodes, restNodes, excludedNodes);

	//Init random sequence generator
	qsrand( QTime::currentTime().msec() );
}

/**
 * @brief  This is the crucial method. Checks if target is a valid pose that is inside limits and not colliking with a known obstacle
 * This method has to be thread-safe if we want to call it from the middleware thread.
 * 
 * @param targetPos robot position in world ref. system
 * @param targetRot robot rotation in world ref system
 * 
 * @return std::pair<bool,QString>: true if target is a valid position for the robot. A diagnosis is provided for debuggin purposes
 */

std::tuple<bool, QString> Sampler::checkRobotValidStateAtTarget(const QVec &targetPos, const QVec &targetRot) const 
{
	QString diagnosis;
	
	//First we move the robot in our copy of innermodel to its current coordinates
 	innerModelSampler->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());

	///////////////////////
	//// Check if the target is a point inside known space and outside forbidden regions
	///////////////////////
	if ( outerRegion.contains( QPointF(targetPos.x(), targetPos.z()) ) == false  )
	{
		diagnosis += "OuterRegion " + QVariant(outerRegion).toString() + "does not contain the point";
		return std::make_tuple(false, diagnosis);
	}
	foreach( const QRectF &r, innerRegions)
	{
		if( r.contains( QPointF(targetPos.x(), targetPos.z())) == true )
		{
			diagnosis += "InnerRegion " + QVariant(r).toString() + "contains the point";
			return std::make_tuple(false, diagnosis);
		}
	}

	///////////////////////
	//// Check if the robot at the target collides with any know object
	///////////////////////
	for ( auto &in : robotNodes )
	{
		for ( auto &out : restNodes )
		{
			if ( innerModelSampler->collide( in, out))
			{
				//qDebug() << __FUNCTION__ << "collision de " << in << " con " << out;
				diagnosis += "Collision of robot's mesh '" + in + "' with '" + out + "' at robot position " + QString::number(targetPos.x()) + ", " + QString::number(targetPos.z());
				return std::make_tuple(false, diagnosis);
			}
		}
	}
	return std::make_tuple(true, diagnosis);
}

/**
 * @brief Expects a 6D vector with translation and rotation values
 * 
 * @param target p_target:...
 * @return std::tuple< bool >
 */
std::tuple< bool, QString > Sampler::checkRobotValidStateAtTarget(const QVec& target) const
{
	if( target.size() != 6 )
		return std::make_tuple(false, QString("Invalid target vector. A 6D vector is required"));
	return checkRobotValidStateAtTarget(target.subVector(0,2), target.subVector(3,5));
}


/**
 * @brief  Ultrafast checkRobotValidStateAtTarget version with simplified functionality. No diagnosisis provided
 * 
 * @param targetPos robot position in world ref. system
 * @param targetRot robot rotation in world ref system
 * 
 * @return bool: true if target is a valid position for the robot. 
 */

bool Sampler::checkRobotValidStateAtTargetFast(const QVec &targetPos, const QVec &targetRot) const 
{
	//First we move the robot in our copy of innermodel to its current coordinates
 	innerModelSampler->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());

	///////////////////////
	//// Check if the robot at the target collides with any know object
	///////////////////////
	for ( auto &in : robotNodes )
	{
		for ( auto &out : restNodes )
		{
			if ( innerModelSampler->collide( in, out))
			{
				//qDebug() << __FUNCTION__ << "collision de " << in << " con " << out;
				return false;
			}
		}
	}
	return true;
}


/**
 * @brief Samples nPoints from robot free space delimited by the union of rectangles in outerregion and 
 * excluding the union of rectangles in innerRegion.
 * 
 * @param nPoints number of points to sample. For a very dense set of points it might take a long time.
 * @return QList< QVec > List of sampled points
 */
QList<QVec> Sampler::sampleFreeSpaceR2(uint nPoints)  
{
	QTime reloj;
 	bool validState = false;
	QVec p,q,res(3,0.f);
	QList<QVec> list;

	for(uint32_t i=0; i<nPoints;i++)
	{
		reloj.start();
		validState = false;
		while( validState == false and reloj.elapsed() < 2000 )
		{
			p =	QVec::uniformVector(1, outerRegion.left(), outerRegion.right());
			q =	QVec::uniformVector(1, outerRegion.bottom(), outerRegion.top());
			QPointF s(p.x(),q.x());
			bool in = false;
			foreach(QRectF rect, innerRegions)
			{
				if( rect.contains(s))
				{
					in = true;
					break;
				}
			}
			if (in == false) 
			{
				res[0] = p.x();
				res[2] = q.x();
				validState = std::get<bool>(checkRobotValidStateAtTarget(res, QVec::zeros(3)));  //ROTATION COULD BE RANDOMIZED HERE
			}
		}
		if( validState )
			list.append(res);
	}
	return list;
}

/**
* @brief Picks a (list of) random point within the limits of a given box and checks that it is out of obstacles (Free Space)
* 
* @return RMat::QVec, a 3D vector with 0 in the Y coordinate
*/
QList<QVec> Sampler::sampleFreeSpaceR2Uniform( const QRectF &box, uint32_t nPoints)  
{
 	bool validState = false;
	QVec p,q, res(3,0.f);
	QList<QVec> list;

	for(uint32_t i=0; i<nPoints;i++)
	{
		while( validState == false )
		{
			p =	QVec::uniformVector(1,box.left(), box.right());
			q =	QVec::uniformVector(1,box.top(), box.bottom());
			QPointF s(p.x(),q.x());
			bool in = false;
			foreach(QRectF rect, innerRegions)
				if( rect.contains(s))
				{
					in = true;
					break;
				}
			if( in == false) 
			{
				res[0] = p.x(); res[2] = q.x();
				validState = std::get<bool>(checkRobotValidStateAtTarget(res, QVec::zeros(3)));
			}
		}
		list.append(res);
		validState = false;
	}
	return list;
}


/**
* @brief Picks a list of random points form a gaussian distribution defined by its mean (meanX, meanY) and diagonal variance and checks 
* that they fall in robot's free space.
* 
* @return RMat::QVec, a 3D vector with 0 in the Y coordinate
*/
QList<QVec> Sampler::sampleFreeSpaceR2Gaussian(float meanX, float meanY, float sigma1, float sigma2, uint32_t nPoints)  
{
 	bool validState = false;
	QVec p,q;
	QList<QVec> list;

	for(uint32_t i=0; i<nPoints;i++)
	{
		while( validState == false )
		{
			p =	QVec::gaussianSamples(1, meanX, sigma1);
			q =	QVec::gaussianSamples(1, meanY, sigma2);
			QPointF s(p.x(),q.x());
			qDebug() << s;
			bool in = false;
			foreach(QRectF rect, innerRegions)
				if( rect.contains(s))
				{
					in = true;
					break;
				}
			if( in == false) 
				validState = std::get<bool>(checkRobotValidStateAtTarget(p, QVec::zeros(3)));
		}
		list.append(QVec::vec3(p.x(),0.f,q.x()));
	}
	return list;
}

/**
 * @brief Provides a free state checker for OMPL. No rotation
 * 
 * @param state ...
 * @return bool
 */
// bool Sampler::isStateValid(const ompl::base::State *state) 
// {
// 	const float x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
// 	const float z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
// 	
// 	innerModelSampler->updateTransformValues("robot", x, 0, z, 0, 0, 0);
// 	
// 	for (uint32_t in=0; in<robotNodes.size(); in++)
// 		for (uint32_t out=0; out<restNodes.size(); out++)
// 			if (innerModelSampler->collide(robotNodes[in], restNodes[out]))
// 			{
// 				return false;
// 			}
// 	return true;
// }

/**
 * @brief Local controller. Goes along a straight line connecting the current robot pose and target pose in world coordinates
 * checking collisions with the environment. It makes the robot advance through a straight line.
 * @param origin Current pose
 * @param target Target pose 
 * @param reachEnd True is successful
 * @param arbol Current search tree
 * @param nodeCurrentPos ...
 * @return RMat::QVec final pose reached
 */
bool Sampler::checkRobotValidDirectionToTarget(const QVec & origin , const QVec & target, QVec &lastPoint)
{
	float stepSize = 100.f; //100 mms chunks  SHOULD BE RELATED TO THE ACTUAL SIZE OF THE ROBOT!!!!!
	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);  
	float step;
	
	//if too close return target
	if (nSteps == 0) 
	{
		lastPoint = target;
		return false;
	}
	step = 1./nSteps;

	//go along visual ray connecting robot pose and target pos in world coordinates. l*robot + (1-r)*roiPos = 0
	QVec point(3);
	float landa = step;
	
	lastPoint = origin;
	for(uint i=1 ; i<=nSteps; i++)
	{
		point = (origin * (1-landa)) + (target * landa);
		if (std::get<bool>(checkRobotValidStateAtTarget(point, QVec::zeros(3))) ) 
		{
			lastPoint  = point;
			landa = landa + step;
		}
		else
			return false;
	}
	return true;
}

/**
 * @brief Constructs the list of robot and world meshes from InnerModel that will be used in detection of collisions
 * 
 * @param node pointer to node used to traverse the tree
 * @param robotId robot's tag name
 * @param inside flag
 * @param in growing list of mesh names belonging to the robot
 * @param out growing list of mesh names belonging to the world
 * @param excluded list of meshes to be excluded from both lists
 * @return void
 */
void Sampler::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded)
{	
	
	if (node->id == robotId)
	{
		inside = true;
	}

	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;
	
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))  
	{
		for (int i=0; i<node->children.size(); i++)
		{
		recursiveIncludeMeshes(node->children[i], robotId, inside, in, out, excluded);
			
		}
		
	}
	
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		if( std::find(excluded.begin(), excluded.end(), node->id) == excluded.end() )			
		{
			if (inside)
			{
				in.push_back(node->id);
			}
			else
				if(mesh or plane)
					out.push_back(node->id);
		}
	}
	
}


/**
 * @brief Checks is there is a valid straight tunnel from origin to target the size of the robot
 * Fast implementation using an extrusion of the bounding box of the robot in the direction of the origin-to-target line
 * 
 * @param origin initial position
 * @param target final position
 * @return bool True is there is free space all along the segment
 */

bool Sampler::checkRobotValidDirectionToTargetOneShot(const QVec & origin , const QVec & target) const
{
	qDebug() << __FUNCTION__ << " Checking between: " << origin << "and " << target;
	
	const float MAX_LENGTH_ALONG_RAY = (target-origin).norm2();
	QVec finalPoint;
	float wRob=420, hRob=1600;  //GET FROM INNERMODEL!!! 
	
	//Compute angle between origin-target line and world Zaxis
	float alfa1 = QLine2D(target, origin).getAngleWithZAxis();
	
	//qDebug() << __FUNCTION__<< "Angle with Z axis" << origin << target << alfa1;
	
	// Update robot's position and align it with alfa1 so it looks at the TARGET point 	
	innerModelSampler->updateTransformValues("robot", origin.x(), origin.y(), origin.z(), 0., alfa1, 0.);
	
	// Compute rotation matrix between robot and world. Should be the same as alfa
	QMat r1q = innerModelSampler->getRotationMatrixTo("world", QString::fromStdString(robotname));
	
	//qDebug() << __FUNCTION__<< "alfa1" << alfa1 << "robot alfa:" << r1q.extractAnglesR_min().y() << "robot tr" << innerModelSampler->transform("world","robot") << MAX_LENGTH_ALONG_RAY; 
	
	// Create a tall box for robot body with center at zero and sides:
	std::shared_ptr<fcl::Box> robotBox(new fcl::Box(wRob, hRob, wRob));

	//Check collision at maximum distance
	float hitDistance = MAX_LENGTH_ALONG_RAY;
	
	//Create and fcl rotation matrix to orient the box with the robot
	const fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	//Resize big box to enlarge it along the ray direction
	robotBox->side = fcl::Vec3f(wRob, hRob, hitDistance);
	
	//Compute the coord of the tip of a "nose" going away from the robot (Z dir) up to hitDistance/2
	const QVec boxBack = innerModelSampler->transformS("world", QVec::vec3(0, hRob/2, hitDistance/2), robotname);
	
	// Create a collision object
	fcl::CollisionObject robotBoxCol(robotBox, R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
	
	
	//qDebug() << __FUNCTION__ << robotBoxCol.getTranslation()[0] <<robotBoxCol.getTranslation()[1]<<robotBoxCol.getTranslation()[2];
	
	
	//Check collision of the box with the world
	for ( auto &it : restNodes)
	{
		if ( innerModelSampler->collide(it, &robotBoxCol))
		{
			qDebug() << __FUNCTION__ << ": Robot collides with " << it;
			return false;
		}
	}
	return true;
}


///UNFiNISHED
bool Sampler::searchRobotValidStateCloseToTarget(QVec& target)
{
	//If current is good, return
	if( std::get<bool>(checkRobotValidStateAtTarget(target,QVec::zeros(3))) == true)
		return true;
	
	target.print("target");
	//Start searching radially from target to origin and adding the vertices of a n regular polygon of radius 1000 and center "target"
	const int nVertices = 12;
	const float radius = 1000.f;
	QVec lastPoint, minVertex, vertex;
	float fi,vert;
	float minDist = radius;
	
	for(int i=0; i< nVertices; i++)
	{
		fi = (2.f*M_PI/nVertices) * i;
		int k;
		bool free;
		for(k=100; k<radius; k=k+100)
		{
			vertex = QVec::vec3(target.x() + k*sin(fi), target.y(), target.z() + k*cos(fi));
			free = std::get<bool>(checkRobotValidStateAtTarget(vertex, QVec::zeros(3)));
			if (free == true) 
				break;
		}
		if( free and k < minDist )
		{
			minVertex = vertex;
			minDist = k;	
			vert = fi;
		}
	}
	if( minDist < radius)
	{
		target = minVertex;
		target.print("new target");
		qDebug() << minDist << vert;
		return true;
	}
	else
		return false;
}


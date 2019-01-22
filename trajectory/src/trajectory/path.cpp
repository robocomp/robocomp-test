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
#include "path.h"

void Path::initialize(const std::shared_ptr<InnerModel> &inner, 
					  const std::shared_ptr<NavigationState> &state_,
					  const std::shared_ptr<RoboCompCommonBehavior::ParameterList> &params)
{
	innerModel = inner;
	state = state_;
	try{ robotname = params->at("RobotName").value;} 
	catch(const std::exception &e){ std::cout << e.what() << " No Robot name defined in config. Using default 'robot' " << std::endl;}
	
	threshold =  QString::fromStdString(params->at("ArrivalTolerance").value).toFloat();
	MINIMUM_SAFETY_DISTANCE =  QString::fromStdString(params->at("MinimumSafetyDistance").value).toFloat(); 
	ROBOT_RADIUS =  QString::fromStdString(params->at("RobotRadius").value).toFloat(); 
	reset();
	std::cout << __FUNCTION__ << "Road initialized correclty" << std::endl;
}


void Path::reloadInnerModel(const std::shared_ptr<InnerModel> &innerModel_)
{
// 	innerModel.reset(innerModel_.get());
	innerModel = innerModel_;
}


//////////////////
/// Main method
/////////////////
void Path::update()
{
	//static QTime reloj = QTime::currentTime();
	
	if(this->isEmpty())
		return;
	
	/// Get robot's position in world and create robot's nose
	QVec robot3DPos = innerModel->transformS("world", robotname);
	auto &primero  = this->first();
	primero = robot3DPos;

	/// Robot nose
	QVec noseInRobot = innerModel->transformS("world", QVec::vec3(0, 0, 1000), robotname);

	//QLine2D nose = QLine2D(QVec::vec2(robot3DPos.x(), robot3DPos.z()), QVec::vec2(noseInRobot.x(), noseInRobot.z()));
	QLine2D nose = QLine2D(QVec::vec2(noseInRobot.x(), noseInRobot.z()),QVec::vec2(robot3DPos.x(), robot3DPos.z()));

	/// Compute closest point in road to robot. If closer than 1000mm it will use the virtual point (tip) instead of the center of the robot.
	Path::iterator closestPoint = computeClosestPointToRobot(robot3DPos);
	
	/// Compute roadTangent at closestPoint
	QLine2D tangent = computeTangentAt(closestPoint);
	setTangentAtClosestPoint(tangent);
	
	/// Compute signed perpenduicular distance from robot to tangent at closest point
	setRobotPerpendicularDistanceToRoad(tangent.perpendicularDistanceToPoint(robot3DPos));
	
	/// Compute signed angle between nose and tangent at closest point
	float ang = nose.signedAngleWithLine2D(tangent);
	if (std::isnan(ang))
		ang = 0;
	setAngleWithTangentAtClosestPoint(ang);
	
	/// Compute distance to target along trajectory
	setRobotDistanceToTarget(computeDistanceToTarget(closestPoint, robot3DPos));  //computes robotDistanceVariationToTarget
	
	setRobotDistanceVariationToTarget(robotDistanceVariationToTarget);
	
	/// Update estimated time of arrival
	setETA();
	
	/// Compute curvature of trajectory at closest point to robot
	setRoadCurvatureAtClosestPoint(computeRoadCurvature(closestPoint, 3));
	
	/// Compute distance to last road point visible with laser field
	setRobotDistanceToLastVisible(computeDistanceToLastVisible(closestPoint, robot3DPos));
	
	/// Compute robot angle in each point
	for( Path::iterator it = this->begin(); it != this->end()-1; ++it )
	{
		QLine2D l = computeTangentAt(it);
		QVec d = l.getDirectionVector();
		float ang =  atan2(d.x(), d.y());
		if(ang>0) ang -= M_PI;
		else ang += M_PI;
		it->rot = QVec::vec3(0, ang, 0);
	}

	/// Check for arrival to target (translation)  TOO SIMPLE
// 	if (((((int) getIndexOfCurrentPoint()+1 == (int) this->size()) and  (getRobotDistanceToTarget() < threshold))) or
// 	    (((int) getIndexOfCurrentPoint()+1 == (int) this->size()) and (getRobotDistanceVariationToTarget() > 0)))
// 	{
// 		qDebug() << "Path::" <<__FUNCTION__ << "ROAD: FINISHED";
// 		qDebug() << "Path::" <<__FUNCTION__ << "	reason: " << ((int) getIndexOfCurrentPoint()+1 == (int)this->size()) << "index " << getIndexOfCurrentPoint();
// 		qDebug() << "Path::" <<__FUNCTION__ << "	reason: " << (getRobotDistanceToTarget() < threshold) << " distance: "<<getRobotDistanceToTarget() << getRobotDistanceVariationToTarget();
// 		
// 		clear();
// 		setFinished(true);
// 	}
//     else
	{
		///////////////////////////////////////////
		//Check for blocked road
		///////////////////////////////////////////
// 		qDebug() << "Path::" <<__FUNCTION__ << "ROAD: Robot distance to last visible" << getRobotDistanceToLastVisible() << (getIterToLastVisiblePoint() < this->end());
		//print();
		if( getRobotDistanceToLastVisible() < 250  and   			//PARAMS
			  getIterToLastVisiblePoint() < this->end())
		{
			qDebug() <<"Path::" <<__FUNCTION__ <<"BLOCKED" << "distanceToLastVisible" << getRobotDistanceToLastVisible();
			setBlocked(true);
		}	
		else
			setBlocked(false);
	}
	
 	//printRobotState(innerModel);
// 	print();
}

	//////////////////////////////////////////////	
	///CHECK ROBOT FOR INMINENT COLLISION. 																						
	///////////////////////////////////////////////
// 	float vside = 0;
// 	int j=0;
// 	road.setBlocked(false);
// 	for(auto i : laserData)
// 	{
// 		if(i.dist < 10) i.dist = 30000;
// 		if( i.dist < baseOffsets[j] + 50 )
// 		{
// 			if( i.angle > -1.30 and i.angle < 1.30)
// 			{
// 				qDebug() << __FUNCTION__<< "Controller: robot stopped to avoid collision because distance to obstacle is less than " << baseOffsets[j] << " "<<i.dist << " " << i.angle;
// 				stopTheRobot(omnirobot_proxy);
// 				road.setBlocked(true);		// MIRAR ESTO
// 				break;
// 			}
// 		}
// 		else
// 		{
// 			if (i.dist < baseOffsets[j] + 150) 
// 			{
// 				if (i.angle > 0)
// 				{
// 					vside  = -80;
// 				}
// 				else
// 				{
// 					vside = 80;
// 				}
// 			}
// 		}
// 		j++;
// 	}


void Path::readRoadFromFile(const std::shared_ptr<InnerModel> &innerModel, std::string name)
{
	clear();
	std::ifstream file(name.c_str(), std::ios_base::in);
	if (file.is_open())
	{
		QVec rPos = innerModel->transformS("world", robotname);
		append(WayPoint(rPos));

		while (file.eof() == false)
		{
			WayPoint w(QVec::zeros(3));
			file >> w.pos[0] >> w.pos[2];
			append(w);
		}
// 		qDebug() << "SpecificWorker::ReadFromFile:: " << QString::fromStdString(name) << "read with" << size() << "points";
	}
	else
		qDebug() << "Could not open File " << QString::fromStdString(name);
}

void Path::readRoadFromList(QList<QVec> list)
{
	Q_ASSERT_X(list.size() > 0, "readRoadFromList", "Empty list");
	clear();
	foreach(QVec point, list)
	{
		append(WayPoint(point));
	}
	//print();
}

void Path::readRoadFromList(const std::list<QVec> &list)
{
	for( auto &point: list)
	{
		append(WayPoint(point));
	}
}

void Path::computeDistancesToNext()
{
	for (int i = 0; i + 1 < this->size(); i++) 
		this->operator[](i).initialDistanceToNext = (this->operator[](i).pos - this->operator[](i + 1).pos).norm2();
}


QLine2D Path::getRobotZAxis(const std::shared_ptr<InnerModel> &innerModel)
{
	QVec robotPos = innerModel->transformS("world", QVec::zeros(3), robotname);
	QVec nose = innerModel->transformS("world", QVec::vec3(0, 0, 1000), robotname);
	return QLine2D(robotPos, nose);
}

float Path::robotDistanceToCurrentPoint(const std::shared_ptr<InnerModel> &innerModel)
{
	return (innerModel->transformS("world", QVec::zeros(3), robotname) - (*this)[indexOfCurrentPoint].pos).norm2();
}

float Path::robotDistanceToNextPoint(const std::shared_ptr<InnerModel> &innerModel)
{
	return (innerModel->transformS("world", QVec::zeros(3), robotname) - (*this)[indexOfNextPoint].pos).norm2();
}

QLine2D Path::getTangentToCurrentPoint()
{
	static QLine2D lineAnt=QLine2D();
	if ((int)(indexOfCurrentPoint  + 1) < size() and size() > 0)
	{
		QVec p1 = QVec::vec2((*this)[indexOfCurrentPoint].pos.x(), (*this)[indexOfCurrentPoint].pos.z());
		QVec p2 = QVec::vec2((*this)[indexOfCurrentPoint + 1].pos.x(), (*this)[indexOfCurrentPoint + 1].pos.z());
		QLine2D line(p1, p2);
		lineAnt = line;
	}
	return lineAnt;
}

QLine2D Path::getTangentToCurrentPointInRobot(const std::shared_ptr<InnerModel> &innerModel)
{
	static QLine2D lineAnt=QLine2D();
	if ((int)(indexOfCurrentPoint  + 1) < size() and size() > 0)
	{
		QVec p1 = QVec::vec3((*this)[indexOfCurrentPoint].pos.x(), 0., (*this)[indexOfCurrentPoint].pos.z());
		QVec p2 = QVec::vec3((*this)[indexOfCurrentPoint + 1].pos.x(), 0., (*this)[indexOfCurrentPoint + 1].pos.z());
		//Use the 3D vector constructor
		QLine2D line(innerModel->transformS(robotname, p1, "world"), innerModel->transformS(robotname, p2, "world"));
		lineAnt = line;
	}
	return lineAnt;
}

void Path::printRobotState(const std::shared_ptr<InnerModel> &innerModel)
{
	QVec robot3DPos = innerModel->transform6D("world", QString::fromStdString(robotname));
	qDebug() << "-------Road status report  ---------------------";
	qDebug() << "	Robot position:" << robot3DPos;
	//qDebug() << "	Target:" << currentTarget.getTranslation();
	qDebug() << "	Target2:" << last().pos <<" Rotation:" << last().hasRotation << last().rot;
	qDebug() << "	Num points:" << this->size();
	qDebug() << "	Robot dist to closest point in road:" << getRobotDistanceToClosestPoint();
	qDebug() << "	Robot perp. dist to road tangent at closest point:" << getRobotPerpendicularDistanceToRoad();
	qDebug() << "	Angle with road:" << getAngleWithTangentAtClosestPoint();
	qDebug() << "	Dist to target:" << getRobotDistanceToTarget();
	qDebug() << "	Distance variation to target:" << getRobotDistanceVariationToTarget();
	qDebug() << "	Road curvature:" << getRoadCurvatureAtClosestPoint();
	//qDebug() << "	Index of closest point:" << getIndexOfClosestPointToRobot();
	qDebug() << "	Closest point:" << getIterToClosestPointToRobot()->pos;
	qDebug() << "	Tangent at closest point:" << getTangentAtClosestPoint();
	qDebug() << "	Current point index: " << indexOfCurrentPoint;
	qDebug() << "	Is Blocked:" << isBlocked();
	qDebug() << "	Is Lost:" << isLost;
	qDebug() << "	Is Finished:" << isFinished();
	qDebug() << "	Requires replanning:" << requiresReplanning;
	qDebug() << "	ETA:" << estimatedTimeOfArrival << " sg";
	qDebug() << "	ElapsedTime:" << elapsedTime / 1000 << " sg";
	qDebug() << "	Duration estimation:" << initialDurationEstimation << " sg";
	qDebug() << "	Estimation error:" << initialDurationEstimation - elapsedTime / 1000 << " sg";
	qDebug() << "	Road BackList size:" << backList.size();


	qDebug() << "----------------------------------------------------";
}

void Path::print() const
{
	qDebug() << __FUNCTION__ << "Printing Road"<<"Size"<<this->size();
	for (int i = 0; i < this->size(); i++)
	{
		const WayPoint &w = (*this)[i];
		qDebug() << "		-"<< i << w.pos << w.rot << i << "Visible" << w.isVisible;
		qDebug() << "		" << "laser ang visibility" << w.visibleLaserAngle << "laser dist visibility"
		         << w.visibleLaserDist << "in robot frame" << w.posInRobotFrame;
	}
}

//////////////////////////////////////////////////////////////////////////////////
///////COMPUTATION OF SCALAR MAGNITUDES OF FORCEFIELD
//////////////////////////////////////////////////////////////////////////////////


/**
 * @brief Computes closest point in trajectory to robot. Updates robotDistanceToClosestPoint and currentPointIndex
 *
 * @param robot 3d vector coding robot's position (x,y,z)
 * @return WayPoint closest to Robot
 */
Path::iterator Path::computeClosestPointToRobot(const QVec &robot)
{
	float min = FLT_MAX;
	QList<WayPoint>::iterator it = this->begin();
	QList<WayPoint>::iterator res = this->end();
	uint count = 0, index = 0;
	
// 	qDebug() << "Path::computeClosestPointToRobot" << this->size();
	if(it == res)
	{
		robotDistanceToClosestPoint = 0.0;
		indexOfCurrentPoint = index;  
		indexOfClosestPointToRobot = index;
		iterToClosestPointToRobot = res; 
		return it;
	}
	
	for (it = this->begin(); it != this->end(); ++it, ++count)
	{
		float d = (it->pos - robot).norm2();
        if (d < min)
        {
            min = d;
            res = it;
            index = count;
        }
	}
    robotDistanceToClosestPoint = min;
    indexOfCurrentPoint = index;  
    indexOfClosestPointToRobot = index;
    iterToClosestPointToRobot = res; 
    return res;
}

QLine2D Path::computeTangentAt(Path::iterator w) const
{
	static QLine2D antLine = QLine2D(QVec::zeros(3), QVec::vec3(0, 0, 1));  //Initial well formed tangent

	if (size() < 2)
	{
		qDebug() << __FUNCTION__ << "Warning. A size 1 road got into here!";
		return antLine;
	}

	Path::iterator ant, post;
	const float MIN_DISTANCE_ALLOWED = 10.f;

    if (w + 1 == this->end())
	{
        ant = w - 1;
        post = w;
    }
    else
    {
		ant = w ;
        post = w + 1;
    }

/*	if (w == this->begin())
		ant = w;
	else
		ant = w - 1;

	if (w + 1 == this->end())
		post = w;
	else
		post = w + 1;
*/
	//Now check if the points are too close
	if ((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED)
		return antLine;

	// The tangent
	//QLine2D l(ant->pos, post->pos);
	QLine2D l(post->pos, ant->pos);
	
	if (std::isnan(l[0]) or std::isnan(l[1]) or std::isnan(l[2]))
	{
		ant->pos.print("ant");
		post->pos.print("post");
		l.print("line");
		print();
		qFatal("Fary in tangent");
	}
	antLine = l;
	return l;
}

/**
 * @brief Computes the distance from the robot to the last visible point in the road
 *
 * @param road ...
 * @param closestPoint ...
 * @param robotPos ...
 * @return float
 */
float Path::computeDistanceToLastVisible(Path::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	Path::iterator it = closestPoint;
	if(it->isVisible == false)
		return 0;
	for (it = closestPoint; it != end() - 1; ++it)
	{
        Path::iterator nextPoint = std::next(it);
		if (nextPoint->isVisible == true)
			dist += (it->pos - nextPoint->pos).norm2();
		else
			break;
	}
	iterToLastVisiblePoint = it+1;
	return dist;
}

/**
 * @brief Computes the distance to the Target along the road starting at closestPoint
 *
 * @param road ...
 * @param robotPos ...
 * @return float Distance to target in the units of InnerModel
 */
float Path::computeDistanceToTarget(Path::iterator closestPoint, const QVec &robotPos)
{		
	float dist = (robotPos - closestPoint->pos).norm2();
	Path::iterator it;
	
	for (it = closestPoint; it != end() - 1; ++it)
	{
		dist += (it->pos - std::next(it)->pos).norm2();
	}
	
	// qDebug()<<"Closest point"<<indexOfClosestPointToRobot<<size();	
	float distE = (robotPos - it->pos).norm2();  //Euclidean distance to last point to detect the robot going away from the target
	robotDistanceVariationToTarget = distE - antDist;
	antDist = distE;
	return dist;
}


/**
 * @brief Computes distance between two points along the road 
 * 
 * @param firstPoint ...
 * @param secondPoint ...
 * @return float distance (-1 if it could not be computed)
 */
float Path::computeDistanceBetweenPointsAlongRoad(Path::iterator firstPoint, Path::iterator secondPoint)
{
	float dist = 0.f;
	for (Path::iterator it = firstPoint; it != secondPoint - 1; ++it)
	{
		dist += (it->pos - (it + 1)->pos).norm2();
	}
	return dist;
}


/**
 * @brief Compute angle between current and next segment using the cross product
 * @param robot2DPos (X,Y) positionof the robot
 * @return float value between 0 curvature for straight lines and PI for U turns.
 */

float Path::computeRoadCurvature(Path::iterator closestPoint, uint pointsAhead)
{
	Path::iterator it, final;
	uint count = 0;
	int achieved = 0;
	float sumAng = 0.f;
	for (it = closestPoint; it != end() - 1 and count < pointsAhead; ++it, ++count, ++achieved)
	{
		float ang = computeTangentAt(it).signedAngleWithLine2D(computeTangentAt(it + 1));
		//qDebug() << achieved << road.size() << ang;
		sumAng += ang;
	}
	if (achieved > 0 and std::isnan(sumAng) == false)
		return sumAng / achieved;
	else
		return 0;
}

/**
 * @brief Estimated time of arrival en miliseconds
 * 
 * @return void
 */
void Path::setETA()
{
	estimatedTimeOfArrival = getRobotDistanceToTarget() / meanSpeed;
	elapsedTime = reloj.elapsed();
}



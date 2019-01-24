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
#include "GenericBase.h"
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	void initialize(int period);
	void cleanPath();
	void updateRobot();
   
private:
	InnerModel *innerModel;
	QGraphicsScene scene;
	QGraphicsView view;
	std::vector<QGraphicsEllipseItem*> points;
	QGraphicsEllipseItem *first, *last;
	QGraphicsPolygonItem *robot;
	//QGraphicsRectItem *box;
	std::vector<QGraphicsRectItem*> boxes;
	QGraphicsPolygonItem *polygon;
	std::vector<QGraphicsLineItem*> lforces;
	struct LData { float dist; float angle;};
	std::vector<LData> laserData;
	QTimer cleanTimer, timerVel;

	const float ROBOT_LENGTH = 50;
	const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.8;
	timeval lastCommand_timeval;
	float advVelx, advVelz, rotVel;
	RoboCompGenericBase::TBaseState bState;
	
	void computeForces();
	void computeLaser(QGraphicsEllipseItem* ellipse, const std::vector<QGraphicsRectItem*> &box);
	void addPoints();
	void cleanPoints();
	void controller();

};

#endif

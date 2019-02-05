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
#include <math.h>
#include "grid.h"
#include "human.h"


// Map
struct TCell
{
   	std::uint32_t id;
    bool free;
    bool visited;
    QGraphicsEllipseItem* g_item;
	float cost;
    
    // method to save the value
    void save(std::ostream &os) const {	os << free << " " << visited; };
    void read(std::istream &is) {	is >> free >> visited ;};
};
		

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
	void controller();
   
private:

	//constants
	const float LEFT = -2500, BOTTOM = -4500, WIDTH = 10000, HEIGHT = 5000;
	const float ROBOT_LENGTH = 400;
	const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;
	const float MAX_LASER_DIST = 4000;
	const float LASER_DIST_STEP = 0.01;
	const int TILE_SIZE = 200;
	const float LASER_ANGLE_STEPS = 50;	
	const float ROBOT_MAX_ADVANCE_SPEED = 600;
	
	InnerModel *innerModel;
	QGraphicsScene scene;
	QGraphicsView view;
	std::vector<QGraphicsEllipseItem*> points;

	QGraphicsEllipseItem *first, *last, *second;
	QGraphicsPolygonItem *robot;
	QGraphicsRectItem *target;
	QGraphicsRectItem *north, *south, *west, *east, *middle;
	
	
	std::vector<QGraphicsItem*> boxes;
	QGraphicsPolygonItem *laser_polygon = nullptr;
	
	// Laser
	struct LData { float dist; float angle;};
	std::vector<LData> laserData;
	QTimer cleanTimer, timerRobot;

	// Robot sim
	timeval lastCommand_timeval;
	float advVelx=0, advVelz=0, rotVel=0;
	RoboCompGenericBase::TBaseState bState;
    RoboCompCommonBehavior::ParameterList params;
	// Grid
	using TDim = Grid<TCell>::Dimensions;
	Grid<TCell> grid;

	// Methods
    void initializeWorld();
	void computeForces();
	void computeLaser(QGraphicsItem *r, const std::vector<QGraphicsItem*> &box);
	void addPoints();
	void cleanPoints();
	void computeVisibility();
	float exponentialFunction(float value, float xValue, float yValue, float min);
	void updateFreeSpaceMap();
	void createPathFromGraph(const std::list<QVec> &path);

	// Target
	struct Target 
	{ 
		QPointF p; 
		bool active = false; 
		QGraphicsRectItem *item;
	};
	Target current_target;

	// This function takes an angle in the range [-3*pi, 3*pi] and wraps it to the range [-pi, pi].
	float rewrapAngleRestricted(const float angle);
  	
protected:
	void mousePressEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
};

#endif

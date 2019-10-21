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
#include <QMainWindow>
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
	void personChangedSlot(Human *human);
   
private:

	//constants
	const float ROBOT_LENGTH = 400;
	const float BALL_MIN = ROBOT_LENGTH/2;
	const float BALL_SIZE = ROBOT_LENGTH;
	const float X_ZERO = 1000;					// dist in mm at which repulsive force vanishes
	const float Y_ZERO = 300;					// max force exerted at ~0 distance from obstacle
	const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 1.6;
	const float MAX_LASER_DIST = 4000;
	const float LASER_DIST_STEP = 0.05;
	const float LASER_ANGLE_STEPS = 50;	
	const float ROBOT_MAX_ADVANCE_SPEED = 600;
	const float ROBOT_MAX_ROTATION_SPEED = 0.9;
	const float FORCE_DISTANCE_LIMIT = (ROBOT_LENGTH * 1.5);  //mm
	const float ROBOT_STEP = (ROBOT_LENGTH * 0.1);
	const float DELTA_H = (ROBOT_LENGTH * 0.1);
	
	InnerModel *innerModel;
	QGraphicsScene scene;
	QGraphicsView view;
	std::vector<QGraphicsEllipseItem*> points;

	QGraphicsEllipseItem *first, *last, *robot_nose, *laser_pose;
	QGraphicsPolygonItem *robot;
	QGraphicsRectItem *target;
	bool active = false;
	
	std::vector<QGraphicsItem*> boxes;
	QGraphicsPolygonItem *laser_polygon = nullptr;
	
	// Laser
	struct LData { float dist; float angle;};
	std::vector<LData> laserData;
	QTimer cleanTimer, timerRobot, controllerTimer;

	// Robot sim
	timeval lastCommand_timeval;
	float advVelx=0, advVelz=0, rotVel=0;
	RoboCompGenericBase::TBaseState bState;
    RoboCompCommonBehavior::ParameterList params;
	QGraphicsPolygonItem* localizationPolygon;
	QGraphicsEllipseItem* spherePolygon;
	int lostMeasure = 0;

	//human
	Human *humanA, *humanB;
	QVector<Human*> human_vector;

	//human Polygon
	// struct PolygonData
	// {
	// 	QString id;
	// 	QPolygonF polygon;
	// 	QGraphicsPolygonItem* item;
	// };
	//QMap<QString, PolygonData> human_poly;
	std::vector<QGraphicsPolygonItem*> human_poly;

//todo: remove when not needed
	//QMap<QString, QGraphicsRectItem *>occupied;	

	// Grid
	using TDim = Grid<TCell>::Dimensions;
	TDim dimensions;
	Grid<TCell> grid;
	
	// Methods
    void initializeWorld();
	void computeForces(const std::vector<QGraphicsEllipseItem*> &path, const std::vector<LData> &lData);
	void computeLaser(QGraphicsItem *r, const std::vector<QGraphicsItem*> &box);
	void addPoints();
	void cleanPoints();
	void computeVisibility(const std::vector<QGraphicsEllipseItem*> &path, const QGraphicsPolygonItem *laser);
	float exponentialFunction(float value, float xValue, float yValue, float min);
	//void updateFreeSpaceMap(QMap<QString, QPolygonF> poly_map);
	void updateFreeSpaceMap(const std::vector<QPolygonF> &new_poly);
	void createPathFromGraph(const std::list<QVec> &path);
	//void markGrid(QGraphicsPolygonItem* poly, bool occupied);

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
	float degreesToRadians(const float angle_);

	//Integrating time
	QTime reloj = QTime::currentTime();


protected:
	void mousePressEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
	void resizeEvent(QResizeEvent *event) override;
};

#endif

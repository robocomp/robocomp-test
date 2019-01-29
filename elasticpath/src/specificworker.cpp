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
#include "specificworker.h"
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <random>
#include <QGridLayout>
#include <QDesktopWidget>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		// RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		// std::string innermodel_path = par.value;
		// innerModel = new InnerModel(innermodel_path);
		// InnerModelTransform *parent = innerModel->getTransform("robot");
		// InnerModelTransform *rawOdometryParentNode = innerModel->newTransform("robot_raw_odometry_parent", "static", parent, 0, 0, 0, 0, 0, 0, 0);
		// InnerModelTransform *rawOdometryNode = innerModel->newTransform("robot_raw_odometry", "static", rawOdometryParentNode,  0, 0, 0, 0, 0, 0, 0);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
	//scene.setSceneRect(-200, -200, 400, 400);
	scene.setSceneRect(LEFT, BOTTOM, WIDTH, HEIGHT);
	view.scale( 1, -1 );
	view.setScene(&scene);
	view.setParent(this);
	QGridLayout* layout = new QGridLayout;
    layout->addWidget(&view);
	this->setLayout(layout);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	//this->resize(view.size());

	// Robot 
	QPolygonF poly2;
	float size = ROBOT_LENGTH/2.f;
	poly2 << QPoint(-size, -size) << QPoint(-size, size) << QPoint(-size/3, size*1.6) << QPoint(size/3, size*1.6) << QPoint(size, size) << QPoint(size, -size);
	QBrush brush;
	brush.setColor(QColor("Orange")); brush.setStyle(Qt::SolidPattern);
	robot = scene.addPolygon(poly2, QPen(QColor("Orange")), brush);
	robot->setPos(0, -2000);
	robot->setRotation(0);
	bState.x = robot->pos().x(); bState.z = robot->pos().y(); bState.alpha = 0;

	// target
	target = scene.addRect(QRectF(-80, -80, 160, 160));
	target->setFlag(QGraphicsItem::ItemIsMovable);
	target->setPos(400, 2000);
	target->setBrush(QColor("LightBlue"));

	// path
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(-200, 200);
	for(auto i: iter::range(-2000, 2000, 250))
	{
		auto ellipse = scene.addEllipse(QRectF(-50,-50, 100, 100), QPen(QColor("LightGreen")), QBrush(QColor("LightGreen")));
		ellipse->setFlag(QGraphicsItem::ItemIsMovable);
		auto y = dis(gen);
		ellipse->setPos(i, y);
		points.push_back(ellipse);
	}

	// bind first and last to robot and target
	first = points.front();
	first->setPos(robot->pos());
	first->setBrush(QColor("MediumGreen"));
	last = points.back();
	last->setPos(target->pos());
	target->setZValue(1);
	
	// Boxes
	auto box = scene.addRect(QRectF(-250,-250,500,500), QPen(Qt::magenta), QBrush(Qt::magenta));
	box->setPos(1500, 1500);
	box->setFlag(QGraphicsItem::ItemIsMovable);
	boxes.push_back(box);

	box = scene.addRect(QRectF(-250,-250, 500,500), QPen(Qt::magenta), QBrush(QColor("magenta")));
	box->setPos(-1600, 1500);
	box->setFlag(QGraphicsItem::ItemIsMovable);
	boxes.push_back(box);

	//Walls
	north = scene.addRect(QRectF(-3500, 0, 7000, 50), QPen(QColor("brown")), QBrush(QColor("brown")));
	north->setPos(0, 3500);
	boxes.push_back(north);
	south = scene.addRect(QRectF(-3500, 0, 7000, 50), QPen(QColor("brown")), QBrush(QColor("brown")));
	south->setPos(0, -3500);
	boxes.push_back(south);
	west = scene.addRect(QRectF(0, -3500, 50, 7000), QPen(QColor("brown")), QBrush(QColor("brown")));
	west->setPos(-3500,0);
	boxes.push_back(west);
	east = scene.addRect(QRectF(0, -3500, 50, 7000), QPen(QColor("brown")), QBrush(QColor("brown")));
	east->setPos(3500,0);
	boxes.push_back(east);

	middle = scene.addRect(QRectF(-3000, 0, 6000, 160), QPen(QColor("brown")), QBrush(QColor("brown")));
	middle->setPos(0,0);
	boxes.push_back(middle);	
	// Laser
	for( auto &&i : iter::range(-M_PI/2.f, M_PI/2.f, M_PI/100.f) )
		laserData.emplace_back(LData{0.f, (float)i});
	
	timer.start(50);
	
	connect(&cleanTimer, &QTimer::timeout, this, &SpecificWorker::cleanPath);
	cleanTimer.start(80);
	//timer.setSingleShot(true);

	advVelz = 0;
	rotVel = 0;

	//timerRobot.setSingleShot(true);
	timerRobot.start(100);
	connect(&timerRobot, &QTimer::timeout, this, &SpecificWorker::updateRobot);
	connect(&timerRobot, &QTimer::timeout, this, &SpecificWorker::controller);
}

// SLOTS
void SpecificWorker::compute()
{
	computeLaser(robot, boxes);
	computeVisibility();
	computeForces();
}

void SpecificWorker::cleanPath()
{
	addPoints();
	cleanPoints();
}

/////////////////////////////////////////////////////////////////////7
/////////
////////////////////////////////////////////////////////////////////

void SpecificWorker::computeVisibility()
{
	const auto &poly = laser_polygon->polygon();
	for(auto &&p: points)
	{
		if( poly.containsPoint(p->pos(), Qt::OddEvenFill))
		{
			p->setData(0, true);
			p->setBrush(QColor("LightGreen"));
		}
		else 
		{
			p->setData(0, false);
			p->setBrush(QColor("DarkGreen"));
		}
	}
}

void SpecificWorker::computeForces()
{
	if(points.size() < 3) return;

	// baselines needed to remove tangential component
	std::vector<QVector2D> base_lines(points.size(), QVector2D(0.f,0.f));
	int k=1;
	for(auto group : iter::sliding_window(points, 3))
	{
		if(group.size()<3) break;

		auto p1 = QVector2D(group[0]->pos());
		auto p3 = QVector2D(group[2]->pos());
		base_lines[k++] = (p1 - p3).normalized();
	}  

	// internal curvature forces
	std::vector<QVector2D> iforces(points.size(), QVector2D(0.f,0.f));
	k=1;
	for(auto group : iter::sliding_window(points, 3))
	{
		if(group.size()<3) break;

		auto p1 = QVector2D(group[0]->pos());
		auto p2 = QVector2D(group[1]->pos());
		auto p3 = QVector2D(group[2]->pos());

		//internal force on p2
		//auto iforce = ((p1-p2)/(p1-p2).length() + (p3-p2)/(p3-p2).length());
		//QVector2D iforce(0,0);
		//if(group[1]->data(0).value<bool>() == true) // inside laser field
		auto iforce = (p1-p2) + (p3-p2);
		iforces[k++] = iforce;
	} 
	
	// external forces
	// we need the minimun distance from each point to the obstacle(s) we compute the closest laser ray tip to each point
	std::vector<QVector2D> eforces;
	static std::vector<QGraphicsLineItem*> lforces;
	
	for(auto &&f: lforces)
		scene.removeItem(f);
	lforces.clear();
	for( auto &&p : points)
	{
		QVector2D force(0,0), f_force(0,0);
		if(p->data(0).value<bool>() == true) // inside laser field
		{
			std::vector<std::tuple<float, QVector2D>> distances;
			std::transform(std::begin(laserData), std::end(laserData), std::back_inserter(distances), [p, this](auto &l)
				{ 
					auto t = QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
					QVector2D tip(robot->mapToScene(t));
					if(QVector2D(t).length() < MAX_LASER_DIST-1)
						return std::make_tuple((QVector2D(p->pos()) - tip).length(), QVector2D(p->pos()) - tip);	
					else
						return std::make_tuple(MAX_LASER_DIST, QVector2D(0,0));	
				});
			auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b){return std::get<float>(a) < std::get<float>(b);});
			force = std::get<QVector2D>(*min);
			float mag = force.length();
			//linear inverse law
			if(mag > 800) mag = 800;
			mag  = -(200.f/800)*mag + 200.f;
			f_force = mag * force.normalized();	
		}
		
		lforces.push_back(scene.addLine(QLineF( p->pos(), p->pos()+f_force.toPointF())));
		eforces.push_back(f_force);
	}
	//qDebug() << "-------";

	//Apply forces to current position
	const float KE = 0.90;
	const float KI = 0.4;	
	//const float KL = 0.06;	
	for(auto &&[point, iforce, eforce, base_line] : iter::zip(points, iforces, eforces, base_lines))
	{
		const auto &idelta = KI * iforce.toPointF();
		QPointF edelta{0,0}; 
		edelta = KE * eforce.toPointF();
		
		const auto force =  idelta + edelta;

		// Removing tangential component
		const auto eprod = QVector2D::dotProduct(QVector2D(force), base_line);
		if(eprod < 5)
		{
			const auto corrected_force = force - (QVector2D::dotProduct(QVector2D(force), base_line)*base_line).toPointF();
			point->setPos( point->pos() + corrected_force ); 
		}
		else
			point->setPos( point->pos() + force ); 
	}
	// reposition endpoints
	first->setPos(robot->pos());
	last->setPos(target->pos());
	second = points[1];
}

////// Add points to the path if needed
void SpecificWorker::addPoints()
{
	std::vector<std::tuple<int, QPointF>> points_to_insert;
	int k=1;
	for(auto &&group: iter::sliding_window(points, 2))
	{
		auto &p1 = group[0];
		auto &p2 = group[1];
		float dist = QVector2D(p1->pos()-p2->pos()).length();
		if (dist > ROAD_STEP_SEPARATION)  
		{
			float l = 0.9 * ROAD_STEP_SEPARATION / dist;   //Crucial que el punto se ponga mas cerca que la condición de entrada
			QLineF line(p1->pos(), p2->pos());
			points_to_insert.push_back(std::make_tuple(k, QPointF{line.pointAt(l)}));
		}
		k++;
	}
	int l=0;
	for(auto &&p : points_to_insert)
	{
		auto r = scene.addEllipse(QRectF(-50,-50,100,100), QPen(QColor("LightGreen")), QBrush(QColor("LightGreen")));
		r->setPos(std::get<QPointF>(p));
		points.insert(points.begin() + std::get<int>(p) + l++, r);
	}
	//qDebug() << points.size();
}

////// Remove points fromthe path if needed
void SpecificWorker::cleanPoints()
{
	std::vector<QGraphicsEllipseItem*> points_to_remove;
	for(const auto &group : iter::sliding_window(points, 2))
	{
		const auto &p1 = group[0];
		const auto &p2 = group[1];
		if(p2 == last)
			break;
		float dist = QVector2D(p1->pos()-p2->pos()).length();
		if (dist < 0.5 * ROAD_STEP_SEPARATION)  
			points_to_remove.push_back(p2);
	}
	for(auto p: points_to_remove)
	{
		scene.removeItem(p);
		points.erase(std::remove_if(points.begin(), points.end(), [p](auto &r){ return p==r;}), points.end());
	}
}

////// Render synthetic laser
void SpecificWorker::computeLaser(QGraphicsItem *r, const std::vector<QGraphicsItem*> &box)
{
	for( auto &&l : laserData )
	{
		l.dist = MAX_LASER_DIST;
		QLineF line(r->pos(), r->mapToScene(QPointF(MAX_LASER_DIST*sin(l.angle), MAX_LASER_DIST*cos(l.angle))));
		for( auto t : iter::range(0.f, 1.f, LASER_DIST_STEP))
		{
			auto r = line.pointAt(t);
			if(std::any_of(std::begin(boxes), std::end(boxes),[r](auto &box){ return box->contains(box->mapFromScene(r));}))
			{
				l.dist = QVector2D(r-line.pointAt(0)).length();
				break;
			}
		}
	}
	if(laser_polygon != nullptr)
		scene.removeItem(laser_polygon);
	QPolygonF poly;
	QBrush brush(QColor("LightPink"), Qt::Dense6Pattern);
	for(auto &&l : laserData)
		poly << r->mapToScene(QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle)));
		
	laser_polygon = scene.addPolygon(poly, QPen(QColor("LightPink")), brush);
}

void SpecificWorker::controller()
{
	// Compute distance to target along path
	float dist_to_target = 0.f;
	for(auto &&g : iter::sliding_window(points, 2))
		dist_to_target += QVector2D(g[1]->pos() - g[0]->pos()).length();
	
	// Check for arrival
	if(dist_to_target < ROBOT_LENGTH)
	{
		advVelz = 0;
		rotVel = 0;
		return;
	}

	// Compute rotation speed. We use angle between robot's nose and line between first and sucessive points
	// as an estimation of curvature ahead
	std::vector<float> angles;
	auto lim = std::min(6, (int)points.size());
	QLineF nose(robot->pos(), robot->mapToScene(QPointF( 0, 50)));
	for(auto &&i: iter::range(1,lim))
		angles.push_back(rewrapAngleRestricted(qDegreesToRadians(nose.angleTo(QLineF(first->pos(),points[i]->pos())))));
	auto min_angle = std::min(angles.begin(), angles.end());	
	rotVel = -0.7 * *min_angle;

	// Compute advance speed
	advVelz = ROBOT_MAX_ADVANCE_SPEED /**	exponentialFunction(1./dist_to_target, 1./700, 0.4, 0.1)*/
									  * exponentialFunction(rotVel, 0.4, 0.4, 0.f);
}

float SpecificWorker::exponentialFunction(float value, float xValue, float yValue, float min)
{
	if( yValue <= 0) return 1.f;
	float landa = -fabs(xValue) / log(yValue);
	float res = exp(-fabs(value)/landa);
	if( res < min )
		return min;
	else
		return res;
}

///Periodic update of robot's state based on its adv and rot speeds.
void SpecificWorker::updateRobot()
{
	// Do nothing if the robot isn't moving
	// if ( (fabs(advVelx)<0.0001 and fabs(advVelz)<0.0001 and fabs(rotVel)<0.0001))
	// 	return;
	
	static QTime reloj = QTime::currentTime();
	QVec incs = QVec::vec3(0, advVelz, rotVel) * ((float)reloj.restart() / 1000.f);
	bState.alpha += incs.z();
	
	QMat m(3,3); 
	m(0,0) = cos(bState.alpha);
	m(0,1) = -sin(bState.alpha);
	m(0,2) = bState.x;
	m(1,0) = sin(bState.alpha);
	m(1,1) = cos(bState.alpha);
	m(1,2) = bState.z;
	m(2,0) = 0;
	m(2,1) = 0;
	m(2,2) = 1;
	
	QVec pos = QVec::vec3(incs.x(), incs.y(), 1);
	//incs.print("incs");
	//pos.print("pos");
	//m.print("m");
	auto r = m * pos;
	bState.x = r.x();
	bState.z = r.y();
	//r.print("r after");
	//qDebug() << "bState after" << bState.x << bState.z << bState.alpha;
	robot->setPos(bState.x, bState.z);
	robot->setRotation(qRadiansToDegrees(bState.alpha));
}


///////////////////////////////777777
/// FUSCA
////////////////////////////////////////


// // internal elongation forces
	// std::vector<QVector2D> lforces(points.size(), QVector2D(0.f,0.f));
	// const float force_cero = 10;
	// k=0;
	// for(auto group : iter::sliding_window(points, 2))
	// {
	// 	if(group.size()<2) break;
	// 	auto p1 = QVector2D(group[0]->pos());
	// 	auto p2 = QVector2D(group[1]->pos());

	// 	// f = -kx  Hook's law
	// 	auto force_mod = (p1-p2).length();
	// 	lforces[k] = (force_mod - force_cero) * ((p2-p1)/force_mod);
	// 	lforces[k+1] = (force_mod - force_cero) * ((p1-p2)/force_mod);
	// 	k++;
	// }  

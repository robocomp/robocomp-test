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
	scene.setSceneRect(-200, -200, 400, 400);
	view.scale( 1, -1 );
	view.setScene(&scene);
	view.setParent(this);
	QGridLayout* layout = new QGridLayout;
    layout->addWidget(&view);
	this->setLayout(layout);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	this->resize(view.size());

	std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(-20, 20);
	for(auto i: iter::range(-180, 180, 25))
	{
		auto ellipse = scene.addEllipse(QRectF(0,0, 10, 10), QPen(Qt::blue), QBrush(Qt::blue));
		ellipse->setFlag(QGraphicsItem::ItemIsMovable);
		auto y = dis(gen);
		ellipse->setPos(i, y);
		points.push_back(ellipse);
		//lforces.push_back(scene.addLine(QLineF(i+5,y,i+5, y), QPen(Qt::red)));
	}
	first = points.front();
	first->setPos(first->x(),0);
	first->setRect(-5,-5, 10, 10);
	first->setBrush(QColor("MediumGreen"));

	last = points.back();
	last->setPos(last->x(),0);
	last->setBrush(QColor("LightBlue"));

	// Boxes
	auto box = scene.addRect(QRectF(0,0, 50,50), QPen(Qt::magenta), QBrush(Qt::magenta));
	box->setPos(0, -100);
	box->setFlag(QGraphicsItem::ItemIsMovable);
	boxes.push_back(box);

	box = scene.addRect(QRectF(0,0, 50,50), QPen(Qt::magenta), QBrush(QColor("brown")));
	box->setPos(100, -150);
	box->setFlag(QGraphicsItem::ItemIsMovable);
	boxes.push_back(box);

	// Laser
	QPolygonF poly;
	for( auto &&i : iter::range(-M_PI/2.f, M_PI/2.f, M_PI/100.f) )
	{
		laserData.emplace_back(LData{0.f, (float)i});
		poly << QPointF(1.f*cos(i), 1.f*sin(i));
	};
	QBrush brush; brush.setColor(QColor("LightPink")); brush.setStyle(Qt::Dense6Pattern);
	polygon = scene.addPolygon(poly, QPen(QColor("LighPink")), brush);
	
	// Robot 
	QPolygonF poly2;
	float size = 8.f;
	poly2 << QPoint(-size, -size) << QPoint(-size, size) << QPoint(-size/3, size*1.6) << QPoint(size/3, size*1.6) << QPoint(size, size) << QPoint(size, -size);
	brush.setColor(QColor("Orange")); brush.setStyle(Qt::SolidPattern);
	robot = scene.addPolygon(poly2, QPen(QColor("Orange")), brush);
	
	robot->setPos(0, -200);
	robot->setRotation(0);
	bState.x = robot->pos().x(); bState.z = robot->pos().y(); bState.alpha = 0;
	
	timer.start(10);
	connect(&cleanTimer, &QTimer::timeout, this, &SpecificWorker::cleanPath);
	cleanTimer.start(100);
	//timer.setSingleShot(true);

	advVelz = 0;
	rotVel = 0;
	//timerRobot.setSingleShot(true);
	timerRobot.start(100);
	connect(&timerRobot, &QTimer::timeout, this, &SpecificWorker::updateRobot);
	//connect(&timerRobot, &QTimer::timeout, this, &SpecificWorker::controller);
	
}

void SpecificWorker::compute()
{
	computeLaser(robot, boxes);
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
		auto iforce = (p1-p2) + (p3-p2);
		iforces[k++] = iforce;
	} 
	
	// external forces
	// we need the minimun distance from each point to the obstacle(s)
	// we compute the closest laser ray tip to each point
	std::vector<QVector2D> eforces;
	for( auto &&p : points)
	{
		std::vector<std::tuple<float, QVector2D>> distances;
		std::transform(std::begin(laserData), std::end(laserData), std::back_inserter(distances), [p, this](auto &l)
			{ 
				QVector2D tip(robot->mapToScene(QPointF(l.dist*cos(l.angle), l.dist*sin(l.angle))));
				return std::make_tuple((QVector2D(p->pos()) - tip).length(), QVector2D(p->pos()) - tip);	
			});
		auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b){return std::get<float>(a) < std::get<float>(b);});
		auto force = std::get<QVector2D>(*min);
		float mag = force.length();
		if(mag > 100) mag = 100;
		mag  = -(10.f/100)*mag + 10.f;
		force = mag * force.normalized();	
		eforces.push_back(force);
	}
	//qDebug() << "-------";

	//Apply forces to current position
	const float KE = 0.90;
	const float KI = 0.2;	
	auto last_pos = last->pos();
	//const float KL = 0.06;	
	for(auto &&[point, iforce, eforce, base_line] : iter::zip(points, iforces, eforces, base_lines))
	{
		const auto &idelta = KI * iforce.toPointF();
		QPointF edelta{0,0}; 
		edelta = KE * eforce.toPointF();
		
		const auto force =  idelta - edelta;

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
	last->setPos(last_pos);
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
		auto r = scene.addEllipse(0,0,10,10);
		r->setPos(std::get<QPointF>(p));
		points.insert(points.begin() + std::get<int>(p) + l++, r);
	}
}

////// Remove points fromthe path if needed
void SpecificWorker::cleanPoints()
{
	std::vector<QGraphicsEllipseItem*> points_to_remove;
	//for(auto &&[group, i] : iter::zip(iter::sliding_window(points, 2), iter::range(1,(int)(points.size()-1))))
	for(const auto &group : iter::sliding_window(points, 2))
	{
		const auto &p1 = group[0];
		const auto &p2 = group[1];
		if(p2 == last)
			break;
		float dist = QVector2D(p1->pos()-p2->pos()).length();
		if (dist < 0.6*ROAD_STEP_SEPARATION)  
			points_to_remove.push_back(p2);
	}
	for(auto p: points_to_remove)
	{
		scene.removeItem(p);
		std::remove_if(points.begin(), points.end(), [p](auto &r){ return p==r;});
	}
}

////// Render synthetic laser
void SpecificWorker::computeLaser(QGraphicsItem *r, const std::vector<QGraphicsRectItem*> &box)
{
	const float MAX_LASER = 300;
	const float LASER_DIST_STEP = 0.01;
	for( auto &&l : laserData )
	{
		l.dist = MAX_LASER;
		QLineF line(r->pos(), r->mapToScene(QPointF(MAX_LASER*sin(l.angle), MAX_LASER*cos(l.angle))));
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
	scene.removeItem(polygon);
	QPolygonF poly;
	QBrush brush; brush.setColor(QColor("LightPink")); brush.setStyle(Qt::Dense6Pattern);
	for(auto &&l : laserData)
		//poly << QPointF(r->pos().x() + l.dist*sin(l.angle), r->pos().y() + l.dist*cos(l.angle));
		poly << r->mapToScene(QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle)));
		
		
	polygon = scene.addPolygon(poly, QPen(QColor("LightPink")), brush);
}

void SpecificWorker::controller()
{
	// Compute rotation speed. We use angle between robot and tangent to line between first and second
	QLineF road(first->pos(), second->pos());
	QLineF nose(robot->pos(), robot->mapToScene(QPointF( 0, 30)));
	float angle = qDegreesToRadians(road.angleTo(nose));
	angle = rewrapAngleRestricted(angle);
	//qDebug() << angle;
	rotVel = 0.7 * angle;

	// Compute advance speed
	float total = 0.f;
	for(auto &&g : iter::sliding_window(points, 2))
		total += QVector2D(g[1]->pos() - g[0]->pos()).length();
	advVelz = total;
	if( advVelz > 20) advVelz = 20;

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

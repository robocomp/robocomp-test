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
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	scene.setSceneRect(-200, -200, 400, 400);
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
	first->setRect(0,0, 20, 20);
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
	box->setPos(100, 150);
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
	
	timer.start(10);
	//timer.setSingleShot(true);
}

void SpecificWorker::compute()
{
	computeLaser(first, boxes);
	computeForces();
	addPoints();
	cleanPoints();
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
		auto p2 = QVector2D(group[1]->pos());
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
				QVector2D tip(first->mapToScene(QPointF(l.dist*cos(l.angle), l.dist*sin(l.angle))));
				return std::make_tuple((QVector2D(p->pos()) - tip).length(), QVector2D(p->pos()) - tip);	
			});
		auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b){return std::get<0>(a) < std::get<0>(b);});
		eforces.emplace_back(std::get<1>(*min));
	}

	//Apply forces to current position
	const float KE = 0.05;
	const float KI = 0.3;	
	auto last_pos = last->pos();
	//const float KL = 0.06;	
	for(auto &&[point, iforce, eforce, base_line] : iter::zip(points, iforces, eforces, base_lines))
	{
		const auto &idelta = KI * iforce.toPointF();
		//const auto &ldelta = KL * lforce.toPointF();

		QPointF edelta{0,0}; 
		if( eforce.length() < 100)
			edelta = KE * eforce.toPointF();
		
		const auto force =  idelta + edelta;

		// Removing tangential component
		const auto corrected_force = force - (base_line * QVector2D::dotProduct(QVector2D(force), base_line)).toPointF();

		//if( QVector2D(force).length() > 1 )	
		point->setPos( point->pos() + force ); 
	}
	// reposition endpoints
	first->setPos(-180,0);
	last->setPos(last_pos);
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
	for(auto &&p : points_to_insert)
	{
		auto r = scene.addEllipse(0,0,10,10);
		r->setPos(std::get<QPointF>(p));
		points.insert(points.begin()+std::get<int>(p), r);
	}
}

////// Remove points fromthe path if needed
void SpecificWorker::cleanPoints()
{
	std::vector<int> points_to_remove;
	int k=1;
	//for(auto &&[group, i] : iter::zip(iter::sliding_window(points, 2), iter::range(1,(int)(points.size()-1))))
	for(const auto &group : iter::sliding_window(points, 2))
	{
		const auto &p1 = group[0];
		const auto &p2 = group[1];
		if(p2 == last)
			break;
		float dist = QVector2D(p1->pos()-p2->pos()).length();
		if (dist < 0.6*ROAD_STEP_SEPARATION)  
			points_to_remove.push_back(k);
		k++;
	}
	qDebug() << "size" << points.size();
	for(auto p: points_to_remove)
	{
		qDebug() << p;
	 	scene.removeItem(*(points.begin()+p));
		qDebug() << "after" << p; 
	 	points.erase(points.begin()+p);
		qDebug() << "after2" << p; 	 
	}
	qDebug() << "--------";
}

////// Render synthetic laser
void SpecificWorker::computeLaser(QGraphicsEllipseItem* ellipse, const std::vector<QGraphicsRectItem*> &box)
{
	const float MAX_LASER = 400;
	const float LASER_DIST_STEP = 0.01;
	for( auto &&l : laserData )
	{
		l.dist = MAX_LASER;
		QLineF line(ellipse->pos(), ellipse->mapToScene(QPointF(MAX_LASER*cos(l.angle), MAX_LASER*sin(l.angle))));
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
		poly << QPointF(ellipse->pos().x()+l.dist*cos(l.angle),ellipse->pos().y()+l.dist*sin(l.angle));
	polygon = scene.addPolygon(poly, QPen(QColor("LightPink")), brush);
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
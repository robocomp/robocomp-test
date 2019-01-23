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

	// Obstacle
	box = scene.addRect(QRectF(0,0, 50,50), QPen(Qt::magenta), QBrush(Qt::magenta));
	box->setPos(0, -100);
	box->setFlag(QGraphicsItem::ItemIsMovable);

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
	computeForces();
	computeLaser(first, box);

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
		//qDebug() << std::get<0>(*min) << std::get<1>(*min);
		eforces.emplace_back(std::get<1>(*min));
	}

	//Apply forces
	const float KE = 0.09;
	const float KI = 0.3;	
	//const float KL = 0.06;	
	for(auto &&[point, iforce, eforce, base_line] : iter::zip(points, iforces, eforces, base_lines))
	{
		const auto &idelta = KI * iforce.toPointF();
		//const auto &ldelta = KL * lforce.toPointF();

		QPointF edelta{0,0}; 
		if( eforce.length() < 100)
			edelta = KE * eforce.toPointF();
		
		//qDebug() << ldelta;
		//const auto force =  edelta + idelta + ldelta;
		const auto force =  idelta + edelta;

		// Removing tangential component
		const auto corrected_force = force - (base_line * QVector2D::dotProduct(QVector2D(force), base_line)).toPointF();

		point->setPos( point->pos() + force ); 
	}
	//qDebug() << "-------------------";
	// reposition endpoints
	first->setPos(-180,0);
	last->setPos(180,0);
}

////// Render synthetic laser
void SpecificWorker::computeLaser(QGraphicsEllipseItem* ellipse, QGraphicsRectItem *box)
{
	const float MAX_LASER = 400;
	for( auto &l : laserData )
	{
		l.dist = MAX_LASER;
		QLineF line(ellipse->pos(), ellipse->mapToScene(QPointF(MAX_LASER*cos(l.angle), MAX_LASER*sin(l.angle))));
		for( auto t :  iter::range(0.f,1.f,0.01f) )
		{
			auto r = line.pointAt(t);
			if( box->contains( box->mapFromScene(r)))
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


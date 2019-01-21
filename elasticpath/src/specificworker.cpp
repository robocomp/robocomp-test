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
    std::uniform_int_distribution<> dis(-30, 30);
	for(auto i: iter::range(-180, 180, 20))
	{
		auto ellipse = scene.addEllipse(QRectF(0,0, 10, 10), QPen(Qt::blue));
		ellipse->setFlag(QGraphicsItem::ItemIsMovable);
		auto y = dis(gen);
		ellipse->setPos(i,y);
		points.push_back(ellipse);
		//lforces.push_back(scene.addLine(QLineF(i+5,y,i+5, y), QPen(Qt::red)));
	}
	first = points.front();
	first->setPos(first->x(),0);
	last = points.back();
	last->setPos(last->x(),0);

	auto box = scene.addRect(QRectF(0,0, 50,50), QPen(Qt::magenta), QBrush(Qt::magenta));
	box->setPos(0, -100);
	box->setFlag(QGraphicsItem::ItemIsMovable);

	timer.start(50);
	//timer.setSingleShot(true);
}

void SpecificWorker::compute()
{
	computeForces();
}

void SpecificWorker::computeForces()
{
	if(points.size() < 3) return;

	std::vector<QVector2D> iforces(points.size(), QVector2D(0.f,0.f));
	std::vector<QVector2D> speed(points.size(), QVector2D(0.f,0.f));
	int k=1;
	for(auto group : iter::sliding_window(points, 3))
	{
		if(group.size()<3) break;

		auto p1 = QVector2D(group[0]->pos());
		auto p2 = QVector2D(group[1]->pos());
		auto p3 = QVector2D(group[2]->pos());

		//internal force on p2
		auto iforce = 0.5*((p1-p2)/(p1-p2).length() + (p3-p2)/(p3-p2).length());
		iforces[k++] = iforce;

		//friction
		//auto friction = 
		//fforces.push_back(friction);

	}  
	for(auto &&[point, force] : iter::zip(points, iforces))
	{
		auto pos = point->pos();
		auto delta = force.toPointF();
		//auto l = lf->line();
		//lf->setLine(pos.x()+5, pos.y(), pos.x()+5, (pos+delta).y());
		point->setPos( pos.x(), (pos + delta).y()); 
		// qDebug() << force;
	}
	first->setPos(points.front()->x(),0);
	last->setPos(points.back()->x(),0);
	// for(auto i=0; i< points.size(); i++)
	// {
	// 	speed[i] = QVector2D(points[i]->pos())- posant[i];
	// }
}


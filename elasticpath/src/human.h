/*
 * Copyright 2018 <copyright holder> <email>
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
 */

#ifndef HUMAN_H
#define HUMAN_H

#include <unordered_map>
#include <iostream> 
#include <fstream>
#include <cppitertools/zip.hpp>
#include <QGraphicsItem>
#include <QPainter>
#include "SocialNavigationGaussian.h"
#include <QGraphicsScene>

using namespace RoboCompSocialNavigationGaussian;

class Human : public QObject, public QGraphicsEllipseItem
{     
	Q_OBJECT
	public:
		Human(const QRectF &r, SocialNavigationGaussianPrxPtr proxy, QGraphicsScene *scene_, QColor color_, QPointF pos);  
		void setPolygon(QGraphicsPolygonItem *poly)				{ polygon_item = poly; }
		QGraphicsPolygonItem * getPolygon() const				{ return polygon_item;}
		void updatePolygon(QPolygonF poly);
	private:
		SocialNavigationGaussianPrxPtr gaussian_proxy;
		QGraphicsPixmapItem* pixmapItem;
		Qt::MouseButton mouseButton;
		QGraphicsEllipseItem *ellipseItem;
		float degreesToRadians(const float angle);
		void updatePolygon(float x, float y, float ang);
		QGraphicsPolygonItem *polygon_item = nullptr;
		QGraphicsScene *scene;
		QColor color;


    protected:
        void mouseReleaseEvent(QGraphicsSceneMouseEvent  *event) override;
		void mousePressEvent(QGraphicsSceneMouseEvent  *event) override;
		void mouseMoveEvent(QGraphicsSceneMouseEvent  *event) override;
	//	QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

	signals:
		void personChangedSignal(Human *human);
		
};

#endif // HUMAN_H

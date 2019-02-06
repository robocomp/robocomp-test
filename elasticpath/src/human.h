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

class Human : public QObject, public QGraphicsEllipseItem
{     
	public:
		Human();
	private:

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent  *event) override;
		QRectF boundingRect() const {};
		void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*){};
		
};

#endif // HUMAN_H
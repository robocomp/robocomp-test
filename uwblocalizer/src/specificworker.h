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

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "serialport.h"
#include <QtSerialPort/QSerialPort>
#include <iostream>
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
	QPointF readData(QSerialPort &serial);

private:
	
	//constants
	const float LEFT = 0, BOTTOM = 0, WIDTH = 5000, HEIGHT = 10000;
	const float ROBOT_LENGTH = 400;

	InnerModel *innerModel;
	SerialPort *sport;
	QSerialPort serial_left, serial_right; 
	QGraphicsScene scene;
	QGraphicsView view;
	QGraphicsPolygonItem *robot;

	template <typename IntegerType>
	IntegerType bitsToInt( const unsigned char* bits, uint init, bool little_endian = true )
	{
		IntegerType result = 0;
		if (little_endian)
			for (int n = sizeof( result ); n >= 0; n--)
			result = (result << 8) + bits[ init + n ];
		else
			for (unsigned n = 0; n < sizeof( result ); n++)
			result = (result << 8) + bits[ n ];
		return result;
	}

	protected:
		void mousePressEvent(QMouseEvent *event) override;

};

#endif

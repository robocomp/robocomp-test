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
#include <QByteArray>
#include <QDesktopWidget>
#include <QGridLayout>
#include <QLineF>

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
	std::cout << __FUNCTION__ << "Initialize worker" << std::endl;

	// scene inizialitation
	resize(QDesktopWidget().availableGeometry(this).size() * 0.4);
	scene.setSceneRect(LEFT, BOTTOM, WIDTH, HEIGHT);
	view.scale( 1, -1 );
	view.setScene(&scene);
	view.setParent(this);
	QGridLayout* layout = new QGridLayout;
    layout->addWidget(&view);
	this->setLayout(layout);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

	//walls
	scene.addRect(QRectF(LEFT, BOTTOM, WIDTH, HEIGHT), QPen(QColor("Brown"), 30));

	// Robot 
	QPolygonF poly2;
	float size = ROBOT_LENGTH/2.f;
	poly2 << QPoint(-size, -size) << QPoint(-size, size) << QPoint(-size/3, size*1.6) << QPoint(size/3, size*1.6) << QPoint(size, size) << QPoint(size, -size);
	QBrush brush;
	brush.setColor(QColor("DarkRed")); brush.setStyle(Qt::SolidPattern);
	robot = scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
	robot->setPos(WIDTH/2, HEIGHT/2);
	robot->setRotation(0);
	robot->setZValue(1);


	serial_left.setPortName("/dev/ttyACM0");
	if(!serial_left.open(QIODevice::ReadWrite))	exit(-1); 
	serial_left.setBaudRate(QSerialPort::Baud115200);

	serial_right.setPortName("/dev/ttyACM1");
	if(!serial_right.open(QIODevice::ReadWrite))	exit(-1); 
	serial_right.setBaudRate(QSerialPort::Baud115200);
	
	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	QPointF posL = readData(serial_left);
	QPointF posR = readData(serial_right);
	robot->setPos((posL+posR)/2.f);
}

QPointF SpecificWorker::readData(QSerialPort &serial)
{
	QByteArray responseData;
	responseData.append((char)2);
    responseData.append((char)0);

	for(int i=0; i<1; i++)
	{
		serial.write(responseData);
  		if (serial.waitForBytesWritten(200)) {}
          //qDebug() << "escrito ok";
		int numRead = 0, numReadTotal = 0;

		char *b;b = new char[18];
		for (;;) 
		{
			if (serial.waitForReadyRead(100)) 
			{
				numRead  = serial.read(b + numReadTotal, serial.bytesAvailable());
				numReadTotal += numRead;
			}
			if(numReadTotal >= 18)
			break;
		}

		// for(int i=0; i<18; i++)
		// 	std::cout << (int)b[i] << " "; 
		// std::cout <<  std::endl;

		if(!(b[0] == 64 and b[1] == 1 and b[2] == 0 and b[3] == 65))
		{	
			qDebug() << "Bad sequence, aborting";
			break;
		}

		int x = bitsToInt<std::int32_t>((unsigned char*)b,5,true);
		int y = bitsToInt<std::int32_t>((unsigned char*)b,9,true);
		int z = bitsToInt<std::int32_t>((unsigned char*)b,13,true);
		//qDebug() << x << y << z;
		//qDebug() << "---------------------";
		delete b;
		serial.flush();
		//usleep(100000);
		return(QPointF(x,y));
	}
}

///////////////////////////////////////////////////////////////////////////////////////7
///// Qt Events
////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::mousePressEvent(QMouseEvent *event)
{
	if(event->button() == Qt::LeftButton)
	{
		auto p = view.mapToScene(event->x(), event->y());	
		qDebug() << __FUNCTION__ << p;
	}
}


/////////////////////////////////////////////////////////////////////////////////////
//int x = (b[8] << 24) | (b[7] << 16) | (b[6] << 8) | (b[5]);
		//int y = (b[12] << 24) | (b[11] << 16) | (b[10] << 8) | (b[9]);
		//int z = (b[16] << 24) | (b[15] << 16) | (b[14] << 8) | (b[13]);
		//int qf = (b[17] << 24) | (0 << 16) | (0 << 8) | (0);
		//qDebug() << x << y << z;
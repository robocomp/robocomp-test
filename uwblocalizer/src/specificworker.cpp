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
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << __FUNCTION__ << "Initialize worker" << std::endl;

	serial.setPortName("/dev/ttyACM0");
	//serial->startSlave("/dev/ttyACM0", 200, "0x02 0x00");
	if(!serial.open(QIODevice::ReadWrite))
		exit(-1); 
	serial.setBaudRate(QSerialPort::Baud115200);

	QByteArray responseData;// = currentRespone.toUtf8();
    responseData.append((char)2);
    responseData.append((char)0);
	serial.write(responseData);
  	if (serial.waitForBytesWritten(200)) 
    {
          qDebug() << "escrito ok";
    }
	int numRead = 0, numReadTotal = 0;
	char b[18];

	for (;;) 
	{
		if (serial.waitForReadyRead(100)) 
        {
			numRead  = serial.read(b + numReadTotal, serial.bytesAvailable());
    		numReadTotal += numRead;
		}
		if(numReadTotal>=18)
		 break;
	}

	for(int i=0; i<18; i++)
	 	std::cout << (int)b[i] << " "; 
	 std::cout <<  std::endl;

	if(b[0] == 61)
		qDebug() << "ok";

	int x = (b[8] << 24) | (b[7] << 16) | (b[6] << 8) | (b[5]);
	int y = (b[12] << 24) | (b[11] << 16) | (b[10] << 8) | (b[9]);
	int z = (b[16] << 24) | (b[15] << 16) | (b[14] << 8) | (b[13]);
	//int qf = (b[17] << 24) | (0 << 16) | (0 << 8) | (0);
	
	
	qDebug() << x << y << z;
	
	int r = bitsToInt<int>(b,5,true);

	this->Period = period;
	//timer.start(Period);
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}




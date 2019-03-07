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

	 //Load World
    initializeWorld();

	//walls
//	scene.addRect(QRectF(LEFT, BOTTOM, WIDTH, HEIGHT), QPen(QColor("Brown"), 30));

	//Axis   
	auto axisX = scene.addLine(QLineF(0, 0, 500, 0), QPen(Qt::red, 50));
	axisX->setPos(0,0);
	boxes.push_back(axisX);
	auto axisZ = scene.addLine(QLineF(0, 0, 0, 500), QPen(Qt::blue, 50));
	axisZ->setPos(0,0);
	boxes.push_back(axisZ);



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
	if(!serial_left.open(QIODevice::ReadWrite))
	{
		std::cout << "Error reading ttyACM0" << std::endl;
 		exit(-1); 
	}
	serial_left.setBaudRate(QSerialPort::Baud115200);

	serial_right.setPortName("/dev/ttyACM1");
	if(!serial_right.open(QIODevice::ReadWrite))
	{
		std::cout << "Error reading ttyACM1" << std::endl;
 		exit(-1); 
	}
	serial_right.setBaudRate(QSerialPort::Baud115200);
	
    //compute initial orientation
    compute_initial_pose(10);

	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute_initial_pose(int ntimes)
{
//	QFile file( "data.txt" );
//	file.open(QIODevice::ReadWrite);
//	QTextStream stream( &file );
	
    QPointF posL, posR;
    for (int cont=0;cont < ntimes;cont++)
    {
        posL = readData(serial_left);
		qPosL.push_back(posL);
        posR = readData(serial_right);
		qPosR.push_back(posR);
        
//		stream << posL.x() << ","<<posL.y() <<","<<posR.x() << ","<<posR.y() << "," <<correctedAngle <<"\n";
//		std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
//	file.close();
//	exit(-1);
//Compute median
    std::vector<QPointF> left = qPosL;
	std::sort(left.begin(), left.end(), [](auto &a,auto &b){return QVector2D(a).length() < QVector2D(b).length();});
	std::vector<QPointF> right = qPosR;
	std::sort(right.begin(), right.end(), [](auto &a,auto &b){return QVector2D(a).length() < QVector2D(b).length();});
	size_t size = left.size();
    if (size % 2 == 0)
	{
	  posL = ((left[size / 2 - 1] + left[size / 2]) / 2);
      posR = ((right[size / 2 - 1] + right[size / 2]) / 2);
	}
	else{
      posL = (left[size / 2]);
	  posR = (right[size / 2]);
	}
	robot->setPos((posL+posR)/2.f);
    initialAngle = QLineF(posR, posL).angle();
    
	std::cout << "Initial IMU orientation ==> " << yaw_class << std::endl;  
	std::cout << "Initial orientation ==> " << initialAngle << std::endl;	  
	std::cout << "Initial orientation offset ==> " << initialAngle-yaw_class << std::endl;
    


}


void SpecificWorker::compute_pose()
{
	QPointF posL, posR;
	//make copy to sort
	std::vector<QPointF> left = qPosL;
	std::sort(left.begin(), left.end(), [](auto &a,auto &b){return QVector2D(a).length() < QVector2D(b).length();});
	std::vector<QPointF> right = qPosR;
	std::sort(right.begin(), right.end(), [](auto &a,auto &b){return QVector2D(a).length() < QVector2D(b).length();});
	size_t size = left.size();
    if (size % 2 == 0)
	{
	  posL = ((left[size / 2 - 1] + left[size / 2]) / 2);
      posR = ((right[size / 2 - 1] + right[size / 2]) / 2);
	}
	else{
      posL = (left[size / 2]);
	  posR = (right[size / 2]);
	}
	robot->setPos((posL+posR)/2.f);
	robot->setRotation(-correctedAngle);
}
void SpecificWorker::compute()
{
	static QPointF pos_ant = robot->pos();
	QPointF posL = readData(serial_left);
	QPointF posR = readData(serial_right);
	qPosL.push_back(posL);
	qPosL.erase(qPosL.begin());
	qPosR.push_back(posR);
	qPosR.erase(qPosR.begin());
	//robot->setPos((posL+posR)/2.f);
	compute_pose();

	if(QVector2D(robot->pos()-pos_ant).length()>50)
	{
		scene.addLine(QLineF(pos_ant, robot->pos()), QPen(QColor("Green"), 50));
		pos_ant = robot->pos();
	}
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


//load world model from file
void SpecificWorker::initializeWorld()
{
    QString val;
    //QFile file(QString::fromStdString(params["World"].value));
	QFile file("robolab.json");
    if(not file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Error reading world file, check config params:" << "robolab.json";
        exit(-1);
    }
    val = file.readAll();
    file.close();
    QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
    QJsonObject jObject = doc.object();
    QVariantMap mainMap = jObject.toVariantMap();
    //load tables
    QVariantMap tables = mainMap[QString("tables")].toMap();
    for (auto &t: tables)
    {
        QVariantList object = t.toList();
        auto box = scene.addRect(QRectF(-object[2].toFloat()/2, -object[3].toFloat()/2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("SandyBrown")), QBrush(QColor("SandyBrown")));
        box->setPos(object[4].toFloat(), object[5].toFloat());
        box->setRotation(object[6].toFloat());
        boxes.push_back(box);
    }
    //load walls
	QVariantMap walls = mainMap[QString("walls")].toMap();
    for (auto &t: walls)
    {
        QVariantList object = t.toList();
        auto box = scene.addRect(QRectF(-object[2].toFloat()/2, -object[3].toFloat()/2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
        box->setPos(object[4].toFloat(), object[5].toFloat());
        box->setRotation(object[6].toFloat());
        boxes.push_back(box);
    }

	//load points
    QVariantMap points = mainMap[QString("points")].toMap();
    for (auto &t: points)
    {
         QVariantList object = t.toList();
         auto box = scene.addRect(QRectF(-object[2].toFloat()/2, -object[3].toFloat()/2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
         box->setPos(object[4].toFloat(), object[5].toFloat());
    //     //box->setRotation(object[6].toFloat()*180/M_PI2);
         boxes.push_back(box);
    }
}

/////////////////////////////////////////////////////////////////////////////////////
//int x = (b[8] << 24) | (b[7] << 16) | (b[6] << 8) | (b[5]);
		//int y = (b[12] << 24) | (b[11] << 16) | (b[10] << 8) | (b[9]);
		//int z = (b[16] << 24) | (b[15] << 16) | (b[14] << 8) | (b[13]);
		//int qf = (b[17] << 24) | (0 << 16) | (0 << 8) | (0);
		//qDebug() << x << y << z;

// IMU SUBSCRIPTION
void SpecificWorker::IMUPub_publish(RoboCompIMU::DataImu imu)
{
	yaw_class = imu.rot.Yaw;
    std::cout<< "angle "<< imu.rot.Yaw << " corrected angle "<<imu.rot.Yaw + initialAngle <<std::endl;
//    std::cout << "pose " << robot->pos()<<std::endl;
	correctedAngle = imu.rot.Yaw + initialAngle;
//	robot->setRotation(qRadiansToDegrees(imu.rot.Yaw + initialAngle));
}

//GENERIC BASE IMPLEMENTATION
void SpecificWorker::GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    state.x = robot->pos().x();
    state.z = robot->pos().y();
  //  QPointF pos(qPosL.back() +qPosR.back()/2.f);
  //  state.correctedX = pos.x();
//	state.correctedZ = pos.y();
    state.alpha = degreesToRadians(-correctedAngle);
}

void SpecificWorker::GenericBase_getBasePose(int &x, int &z, float &alpha)
{
	x = robot->pos().x();
    z = robot->pos().y();
    alpha = degreesToRadians(-correctedAngle);
}

//UTILITIES
float SpecificWorker::degreesToRadians(const float angle_)
{	
	float angle = angle_ * 2*M_PI / 360;
	if(angle > M_PI)
   		return angle - M_PI*2;
	else if(angle < -M_PI)
   		return angle + M_PI*2;
	else return angle;
}


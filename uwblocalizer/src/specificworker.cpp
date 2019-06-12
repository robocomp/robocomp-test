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
//#include <QDesktopWidget>
//#include <QGridLayout>
//#include <QLineF>

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

	dbuffer.init(myconverter);

// 	// scene inizialitation
// 	resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
// 	scene.setSceneRect(LEFT, BOTTOM, WIDTH, HEIGHT);
// 	view.scale( 1, -1 );
// 	view.setScene(&scene);
// 	view.setParent(this);
// 	QGridLayout* layout = new QGridLayout;
//     layout->addWidget(&view);
// 	this->setLayout(layout);
// 	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
// 
// 	 //Load World
//     initializeWorld();
// 
// 	//walls
// 	//	scene.addRect(QRectF(LEFT, BOTTOM, WIDTH, HEIGHT), QPen(QColor("Brown"), 30));
// 
// 	//Axis   
// 	auto axisX = scene.addLine(QLineF(0, 0, 500, 0), QPen(Qt::red, 50));
// 	axisX->setPos(0,0);
// 	boxes.push_back(axisX);
// 	auto axisZ = scene.addLine(QLineF(0, 0, 0, 500), QPen(Qt::blue, 50));
// 	axisZ->setPos(0,0);
// 	boxes.push_back(axisZ);
// 
// 	// Robot 
// 	QPolygonF poly2;
// 	float size = ROBOT_LENGTH/2.f;
// 	poly2 << QPoint(-size, -size) << QPoint(-size, size) << QPoint(-size/3, size*1.6) << QPoint(size/3, size*1.6) << QPoint(size, size) << QPoint(size, -size);
// 	QBrush brush;
// 	brush.setColor(QColor("DarkRed")); brush.setStyle(Qt::SolidPattern);
// 	robot = scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
// 	robot->setPos(WIDTH/2, HEIGHT/2);
// 	robot->setRotation(0);
// 	robot->setZValue(-10);
// 
// 	// Robot estable
// 	poly2.clear();
// 	poly2 << QPoint(-size, -size) << QPoint(-size, size) << QPoint(-size/3, size*1.6) << QPoint(size/3, size*1.6) << QPoint(size, size) << QPoint(size, -size);
// 	brush.setColor(QColor("DarkGreen")); brush.setStyle(Qt::SolidPattern);
// 	robot_stable = scene.addPolygon(poly2, QPen(QColor("DarkGreen")), brush);
// 	robot_stable->setPos(WIDTH/2, HEIGHT/2);
// 	robot_stable->setRotation(0);
// 	robot_stable->setZValue(-10);
	
	serial_left.setPortName("/dev/ttyACM0");
	if(!serial_left.open(QIODevice::ReadWrite))
	{
		std::cout << "Error reading ttyACM0" << std::endl;
 		exit(-1); 
	}
	serial_left.setBaudRate(QSerialPort::Baud115200);
    	std::cout << "UWB Radio opened succesfully" << std::endl;

	// serial_right.setPortName("/dev/ttyACM1");
	// if(!serial_right.open(QIODevice::ReadWrite))
	// {
	// 	std::cout << "Error reading ttyACM1" << std::endl;
 	// 	exit(-1); 
	// }
	// serial_right.setBaudRate(QSerialPort::Baud115200);
	
    //compute initial pose
   //compute_initial_pose(50);
	
	//init_kalman();
	this->Period = 100;
	timer.start(Period);
	// GPS sensor
	//connect(&sensor_timer, &QTimer::timeout, this, &SpecificWorker::filteredSensor);
	//sensor_timer.start(1000);
}

// void SpecificWorker::compute_initial_pose(int ntimes)
// {
// 	QPointF posL, posR;
//     for (int cont=0;cont < ntimes;cont++)
//     {
// 		qPosL.push_back(posL);
// 		xMedL += posL.x();
// 		yMedL += posL.y();
//         //posR = readData(serial_right);
// 		xMedR += posR.x();
// 		yMedR += posR.y();
// 		qPosR.push_back(posR);
// 		std::cout<<posL.x() << " "<< posL.y()<<" " <<posR.x() << " "<< posR.y()<<std::endl;
// 		std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 	}
// 
// 	// media
// 	xMedL /= ntimes;
// 	yMedL /= ntimes;
// 	xMedR /= ntimes;
// 	yMedR /= ntimes;
// 	
// 	// Dist media
// 	float distL = sqrt(xMedL*xMedL + yMedL*yMedL);
// 	float distR = sqrt(xMedR*xMedR + yMedR*yMedR);
// 	
// 	// vector sort
// 	std::vector<QPointF> left = qPosL;
// 	std::sort(left.begin(), left.end(), [distL](auto &a,auto &b){return (QVector2D(a).length()-distL) < (QVector2D(b).length()-distL);});
// 	std::vector<QPointF> right = qPosR;
// 	std::sort(right.begin(), right.end(), [distR](auto &a,auto &b){return (QVector2D(a).length()-distR) < (QVector2D(b).length()-distR);});
// 	for (int i=0;i<ntimes;i++)
// 	{
// 		continue;
// 	}	
// 
// 	// median
// 	size_t size = left.size();
//     if (size % 2 == 0)
// 	{
// 	  posL = ((left[size / 2 - 1] + left[size / 2]) / 2);
//       posR = ((right[size / 2 - 1] + right[size / 2]) / 2);
// 	}
// 	else{
//       posL = (left[size / 2]);
// 	  posR = (right[size / 2]);
// 	}
// 	
// 	qDebug()<< "center point"<<posL<<posR;	
// 	initialAngle = QLineF(posR, posL).angle();
// 	if (yaw_class == -999999)
// 	{
// 		std::cout << "Error waiting imu value exiting" << std::endl;
// 		exit(0);
// 	}
// 	std::cout << "Initial IMU orientation ==> " << yaw_class << std::endl;  
// 	std::cout << "Initial orientation ==> " << initialAngle << std::endl;	  
// 	std::cout << "Initial orientation offset ==> " << initialAngle-yaw_class << std::endl;
// 	initialAngle -= yaw_class;
// 
// 	robot->setPos((posL+posR)/2.f);
// 
// }

// void SpecificWorker::compute_pose()
// {
// 	QPointF posL, posR;
// 	//make copy to sort
// 	std::vector<QPointF> left = qPosL;
// 	std::sort(left.begin(), left.end(), [](auto &a,auto &b){return QVector2D(a).length() < QVector2D(b).length();});
// 	std::vector<QPointF> right = qPosR;
// 	std::sort(right.begin(), right.end(), [](auto &a,auto &b){return QVector2D(a).length() < QVector2D(b).length();});
// 	size_t size = left.size();
//     if (size % 2 == 0)
// 	{
// 	  posL = ((left[size / 2 - 1] + left[size / 2]) / 2);
//       posR = ((right[size / 2 - 1] + right[size / 2]) / 2);
// 	}
// 	else{
//       posL = (left[size / 2]);
// 	  posR = (right[size / 2]);
// 	}
// 	robot->setPos((posL+posR)/2.f);
// 	robot->setRotation(-correctedAngle);
// }
// 
// void SpecificWorker::init_kalman()
// {
// //	QPointF posL = readData(serial_left);
// 	QPointF posL(2500,5000);
// 	//  Initial position
// 	x.x() = posL.x();
// 	x.y() = posL.y();
// 	x.theta() = yaw_class;
// 	
// 	//Covarianza sys
// 	Kalman::Covariance<State> sC(3,3);
// 	sC.setZero();
// 	// sC(0,0) = 0.55;
// 	// sC(1,1) = 0.55;
// 	// sC(2,2) = 0.1;
// 
// 	sC(0,0) = 0.00001;
// 	sC(1,1) = 0.00001;
// 	sC(2,2) = 0.00001;
// 
// 	sys.setCovariance(sC);
// 	
// 	//positionModel
// 	Kalman::Covariance<PositionMeasurement> pC(2,2);
// 	pC.setZero();
// 	
// 	pC(0,0) = 2;
// 	pC(1,1) = 2;
// 
// 	pm.setCovariance(pC);
// 	
// 	//orientationModel
// 	Kalman::Covariance<OrientationMeasurement> oC(1,1);
// 	//oC(0,0) = 0.1;
// 	oC(0,0) = 0.1;
// 	om.setCovariance(oC);
// 
// 	// Init filters with true system state
// 	//ukf = Kalman::UnscentedKalmanFilter<State>(1);
//     predictor.init(x);
// 	ekf.init(x);
// 	robot->setPos(posL);
// 
// 	generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
// 	noise = std::normal_distribution<float>(0, 1);
// }

void SpecificWorker::compute()
{
	static QTime reloj = QTime::currentTime();
	
	posL = readData(serial_left);
	for(auto &&d: posL)
        	qDebug() << d;
	qDebug() << "---------------";
	dbuffer.put(readData(serial_left), 0);
    
	//posR = readData(serial_right);

//	qPosL.push_back(posL);
//	qPosL.erase(qPosL.begin());
//	qPosR.push_back(posR);
//	qPosR.erase(qPosR.begin());

	// try
	// {
	// 	differentialrobot_proxy->getBaseState(bState);
	// }
	// catch(...)
	// {
	// 	std::cout<< "Error reading bState from differentialRobot" <<std::endl;
	// }

// 	//kalman
	
// 	float time = ((float)reloj.restart() / 1000.f);
// 	u.v() = 200*time;//bState.advVx * time;
// 	u.dtheta() = 0.05*time;//-bState.rotV * time;

// 	State x2;
// /*	x2.x() = posL.x();
// 	x2.y() = posL.y();
// 	x2.theta() += 0.1*time;
// */
// 	auto x_pred = predictor.predict(sys, u);

// 	x2.x() =  x_pred.x() + 50 *noise(generator);
// 	x2.y() =  x_pred.y() + 50 *noise(generator);
// 	x2.theta() = x_pred.theta() + 0.1 *noise(generator);;

//     auto x_ekf = ekf.predict(sys, u);
// //	auto x_ukf = ukf.predict(sys, u);
// 	OrientationMeasurement orientation = om.h(x2);
// 	x_ekf = ekf.update(om, orientation);
// //	x_ukf = ukf.update(om, orientation);
// 	PositionMeasurement position = pm.h(x2);
// 	x_ekf = ekf.update(pm, position);
// //	x_ukf = ukf.update(pm, position);


	// if(QVector2D(robot->pos()-pos_ant).length()>50)
	// {
	// 	scene.addLine(QLineF(pos_ant, robot->pos()), QPen(QColor("Green"), 50));
	// 	pos_ant = robot->pos();
		
//		scene.addLine(QLineF(pos_antEKF, QPointF(x_ekf.x(),x_ekf.y())), QPen(QColor("Blue"), 50));
//		pos_antEKF = QPointF(x_ekf.x(),x_ekf.y());
		
		//		scene.addLine(QLineF(pos_antUKF, QPointF(x_ukf.x(),x_ukf.y())), QPen(QColor("LightRed"), 50));
		//		pos_antUKF = QPointF(x_ukf.x(),x_ukf.y());

//		scene.addLine(QLineF(pos_antPre, QPointF(x_pred.x(),x_pred.y())), QPen(QColor("Orange"), 50));
//		pos_antPre = QPointF(x_pred.x(),x_pred.y());
		
	//	scene.addLine(QLineF(bState_ant, QPointF(bState.x, bState.z)), QPen(QColor("Blue"), 50));
	//	bState_ant = QPointF(bState.x, bState.z);

//	}
	//robot->setPos(QPointF(x2.x(), x2.y()));
	//robot->setRotation(qRadiansToDegrees(x2.theta()));
	
	// // second robot
	// robot_stable->setPos(QPointF(x_ekf.x(), x_ekf.y()));
	// robot_stable->setRotation(qRadiansToDegrees(x_ekf.theta()));
	
	// // Draw P
	// p_circle->setRect(0,0,100*P(0,0),100*P(1,1));
	// p_circle->setPos(QPointF(x_ekf.x(), x_ekf.y()));
	// p_circle->setRotation(qRadiansToDegrees(x_ekf.theta()));

}

// /// Every second the timer calls this method
// void SpecificWorker::filteredSensor()
// {
// 	qDebug() << "gola";
// 	//filter accumulated data
// 	State mean;
// 	std::vector<State> data = sensor_data;
// 	mean = std::accumulate(data.begin(), data.end(), mean, [](auto &m, auto &d){ return m + d;}) / data.size();
// 	std::transform(data.begin(), data.end(), data.begin(), 
// 		[mean](auto &a){ a.x() = a.x()-mean.x(); a.y() = a.y()-mean.y(); return a;});
// 	std::nth_element(data.begin(), data.begin() + data.size() / 2, data.end(), 
// 		[](auto &a, auto &b){return a.xymod() < b.xymod();});
// 	State s = sensor_data.at(sensor_data.size()/2);
// 	std::cout << s << std::endl;
	
// 	//update the filter with the result
	
// 	OrientationMeasurement orientation = om.h(s);
// 	this->x_ekf = ekf.update(om, orientation);
// 	PositionMeasurement position = pm.h(s);
// 	x_ekf = ekf.update(pm, position);

// 	sensor_data.clear();
// }

std::vector<int> SpecificWorker::readData(QSerialPort &serial)
{
	QByteArray responseData;
	responseData.append((char)2);
    responseData.append((char)0);

	// for(int i=0; i<1; i++)
	// {
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
			qFatal("Bas sequence");
		}

		int x = bitsToInt<std::int32_t>((unsigned char*)b,5,true);
		int y = bitsToInt<std::int32_t>((unsigned char*)b,9,true);
		int z = bitsToInt<std::int32_t>((unsigned char*)b,13,true);
		//qDebug() << x << y << z;
		//qDebug() << "---------------------";
		delete b;
		serial.flush();
		//usleep(100000);
		return std::vector<int>{x,y,z};
//	}
}

///////////////////////////////////////////////////////////////////////////////////////7
///// FullPose Interface
////////////////////////////////////////////////////////////////////////////////////////

FullPose SpecificWorker::FullPoseEstimation_getFullPose()
{
	FullPose fp;
	dbuffer.get(fp);
	return fp;
}

///////////////////////////////////////////////////////////////////////////////////////7
///// Qt Events
////////////////////////////////////////////////////////////////////////////////////////

// void SpecificWorker::mousePressEvent(QMouseEvent *event)
// {
// 	if(event->button() == Qt::LeftButton)
// 	{
// 		auto p = view.mapToScene(event->x(), event->y());	
// 		qDebug() << __FUNCTION__ << p;
// 	}
// }
// 
// 
// //load world model from file
// void SpecificWorker::initializeWorld()
// {
//     QString val;
//     //QFile file(QString::fromStdString(params["World"].value));
// 	QFile file("robolab.json");
//     if(not file.open(QIODevice::ReadOnly | QIODevice::Text))
//     {
//         qDebug() << "Error reading world file, check config params:" << "robolab.json";
//         exit(-1);
//     }
//     val = file.readAll();
//     file.close();
//     QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
//     QJsonObject jObject = doc.object();
//     QVariantMap mainMap = jObject.toVariantMap();
//     //load tables
//     QVariantMap tables = mainMap[QString("tables")].toMap();
//     for (auto &t: tables)
//     {
//         QVariantList object = t.toList();
//         auto box = scene.addRect(QRectF(-object[2].toFloat()/2, -object[3].toFloat()/2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("SandyBrown")), QBrush(QColor("SandyBrown")));
//         box->setPos(object[4].toFloat(), object[5].toFloat());
//         box->setRotation(object[6].toFloat());
//         boxes.push_back(box);
//     }
//     //load walls
// 	QVariantMap walls = mainMap[QString("walls")].toMap();
//     for (auto &t: walls)
//     {
//         QVariantList object = t.toList();
//         auto box = scene.addRect(QRectF(-object[2].toFloat()/2, -object[3].toFloat()/2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
//         box->setPos(object[4].toFloat(), object[5].toFloat());
//         box->setRotation(object[6].toFloat());
//         boxes.push_back(box);
//     }
// 
// 	//load points
//     QVariantMap points = mainMap[QString("points")].toMap();
//     for (auto &t: points)
//     {
//          QVariantList object = t.toList();
//          auto box = scene.addRect(QRectF(-object[2].toFloat()/2, -object[3].toFloat()/2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
//          box->setPos(object[4].toFloat(), object[5].toFloat());
//     //     //box->setRotation(object[6].toFloat()*180/M_PI2);
//          boxes.push_back(box);
//     }
// }

/////////////////////////////////////////////////////////////////////////////////////
//int x = (b[8] << 24) | (b[7] << 16) | (b[6] << 8) | (b[5]);
		//int y = (b[12] << 24) | (b[11] << 16) | (b[10] << 8) | (b[9]);
		//int z = (b[16] << 24) | (b[15] << 16) | (b[14] << 8) | (b[13]);
		//int qf = (b[17] << 24) | (0 << 16) | (0 << 8) | (0);
		//qDebug() << x << y << z;

// IMU SUBSCRIPTION
void SpecificWorker::IMUPub_publish(RoboCompIMU::DataImu imu)
{
// 	yaw_class = imu.rot.Yaw;
// //    std::cout<< "angle "<< imu.rot.Yaw << " corrected angle "<<imu.rot.Yaw + initialAngle <<std::endl;
// //    std::cout << "pose " << robot->pos()<<std::endl;
// 	correctedAngle = imu.rot.Yaw;// + initialAngle;
// //	robot->setRotation(qRadiansToDegrees(imu.rot.Yaw + initialAngle));
}

//GENERIC BASE IMPLEMENTATION
void SpecificWorker::GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state)
{
//     state.x = robot->pos().x();
//     state.z = robot->pos().y();
//   //  QPointF pos(qPosL.back() +qPosR.back()/2.f);
//   //  state.correctedX = pos.x();
// //	state.correctedZ = pos.y();
//     state.alpha = degreesToRadians(-correctedAngle);
// 	state.alpha = 0;
}

void SpecificWorker::GenericBase_getBasePose(int &x, int &z, float &alpha)
{
// 	x = robot->pos().x();
//     z = robot->pos().y();
//     alpha = degreesToRadians(-correctedAngle);
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

// RESIZE
// zoom
// void SpecificWorker::wheelEvent(QWheelEvent *event)
// {
// 	const QGraphicsView::ViewportAnchor anchor = view.transformationAnchor();
// 	view.setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
// 	int angle = event->angleDelta().y();
// 	qreal factor;
// 	if (angle > 0) 
// 	{
// 		factor = 1.1;
// 		QRectF r = scene.sceneRect();
// 		this->scene.setSceneRect(r);
// 	}
// 	else
// 	{
// 		factor = 0.9;
// 		QRectF r = scene.sceneRect();
// 		this->scene.setSceneRect(r);
// 	}
// 	view.scale(factor, factor);
// 	view.setTransformationAnchor(anchor);
// 
// 	QSettings settings("RoboComp", "UWBLocalizer");
// 	settings.beginGroup("QGraphicsView");
// 		settings.setValue("matrix", view.transform());
// 	settings.endGroup();
// }
// 
// void SpecificWorker::resizeEvent(QResizeEvent *event)
// {
// 	QSettings settings("RoboComp", "UWBLocalizer");
// 	settings.beginGroup("MainWindow");
// 		settings.setValue("size", event->size());
// 	settings.endGroup();
// }

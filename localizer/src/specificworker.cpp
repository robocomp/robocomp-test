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
	std::cout << "Initialize worker" << std::endl;

	// scene inizialitation
	resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
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
	brush.setColor(QColor("Magenta")); brush.setStyle(Qt::SolidPattern);
	robot = scene.addPolygon(poly2, QPen(QColor("Magenta")), brush);
	robot->setPos(WIDTH/2, HEIGHT/2);
	robot->setRotation(0);
	robot->setZValue(-10);

	try
	{
		auto poseUWB = fullposeestimation1_proxy->getFullPose();
		qDebug() << "UWB:" << poseUWB.x << poseUWB.y << poseUWB.z;
		robot->setPos(poseUWB.x, poseUWB.y);
		initX = poseUWB.x;
		initZ = poseUWB.y;  //CHANGE IN ORIGIN
		qDebug() << "robot INIT:" << robot->pos();

	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}

	this->Period = 100;
	timer.start(Period);
}

void SpecificWorker::compute()
{

 	try
 	{
		 auto pose_real_sense = fullposeestimation_proxy->getFullPose();
		 //qDebug() << "realsense:" << pose_real_sense.x << pose_real_sense.y << pose_real_sense.z << pose_real_sense.ry;

		robot->setPos(initX + pose_real_sense.x, initZ + pose_real_sense.z);
		robot->setRotation(qRadiansToDegrees(pose_real_sense.ry));
		qDebug() << "robot:" << robot->pos();

 	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading " << e << std::endl;
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


// RESIZE
// zoom
void SpecificWorker::wheelEvent(QWheelEvent *event)
{
	const QGraphicsView::ViewportAnchor anchor = view.transformationAnchor();
	view.setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	int angle = event->angleDelta().y();
	qreal factor;
	if (angle > 0) 
	{
		factor = 1.1;
		QRectF r = scene.sceneRect();
		this->scene.setSceneRect(r);
	}
	else
	{
		factor = 0.9;
		QRectF r = scene.sceneRect();
		this->scene.setSceneRect(r);
	}
	view.scale(factor, factor);
	view.setTransformationAnchor(anchor);

	QSettings settings("RoboComp", "UWBLocalizer");
	settings.beginGroup("QGraphicsView");
		settings.setValue("matrix", view.transform());
	settings.endGroup();
}

void SpecificWorker::resizeEvent(QResizeEvent *event)
{
	QSettings settings("RoboComp", "UWBLocalizer");
	settings.beginGroup("MainWindow");
		settings.setValue("size", event->size());
	settings.endGroup();
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
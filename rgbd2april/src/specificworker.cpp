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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	rgbd_proxies.push_back(rgbd_proxy);
	rgbd_proxies.push_back(rgbd1_proxy);
	rgbd_proxies.push_back(rgbd2_proxy);
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
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innermodel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }



	


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	//initialize cameras
	std::vector<std::string> t_camera_names, camera_names;
	t_camera_names = {"cam1Translation", "cam2Translation", "cam3Translation"};
	camera_names = {"camera1", "camera2", "camera3"};

	cameras.push_back(QVec::vec6(-736.722, -887.987, 3629.57, -1.01273, -0.00594977, -0.0250499 ));
	cameras.push_back(QVec::vec6( -166.563, -1072.82, 3817.9, 0.052354, 0.969563, -1.61103 ));
	cameras.push_back(QVec::vec6(769.173, -1202.67, 4027.94, 1.17315, -0.0165724, 3.13605));
	for(auto&& [name, t_name, cam] : iter::zip(camera_names, t_camera_names, cameras))
	{
//		QVec t_pose = updateCameraPosition(t_name, cam);
	}

	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	RoboCompJointMotor::MotorStateMap hState;
	RoboCompGenericBase::TBaseState bState;
	RoboCompRGBD::ColorSeq color;
	for (int camera = 0;camera<3;camera++)
	{
		try{
			rgbd_proxies[camera]->getRGB(color, hState,bState);
			
			computeApril(color, camera);
		}catch(...)
		{
			qDebug()<<"Error connectng to camera: "<<camera;
		}
	}
}

void SpecificWorker::computeApril(RoboCompRGBD::ColorSeq color, int camera)
{
	try
	{
		RoboCompAprilTagsServer::Image aprilImage;
		aprilImage.frmt.modeImage=RoboCompAprilTagsServer::RGB888Packet;
		aprilImage.frmt.width=640;
		aprilImage.frmt.height=480;
		aprilImage.data.resize(aprilImage.frmt.width * aprilImage.frmt.height *3 );
		
		memcpy(&aprilImage.data[0], &color[0], 640*480*3);
	cv::Mat frame(480, 640, CV_8UC3,  &(aprilImage.data)[0]);
	cv::cvtColor(frame, frame, CV_BGR2RGB);
	cv::line(frame,cv::Point(320,0),cv::Point(320,480),cv::Scalar(255,0,0),2 );	
	cv::line(frame,cv::Point(0,240),cv::Point(640,240),cv::Scalar(255,0,0),2 );
	cv::flip(frame,frame,1);
	cv::imshow(to_string(camera), frame);
	cv::setMouseCallback(to_string(camera), mouse_callback, this);
		RoboCompAprilTagsServer::tagsList tags = apriltagsserver_proxy->getAprilTags(aprilImage, 348, 535.4, 539.2);
		if(tags.size() > 0)
		{
			for (auto tag: tags)
			{
//				QVec p = QVec::vec3(tag.tx, tag.ty, tag.tz);
//				QVec w = innermodel->transform("world", p, QString::fromStdString(camera));

//				std::cout<<"Position "<<tag.tx <<";"<< tag.ty << ";" << tag.tz <<";"<<std::endl;
//				std::cout<<"World "<< w.x() <<";"<< w.y() << ";" << w.z() <<";"<<std::endl;
				std::string result = QString::number(camera).toStdString();
				//to get apriltag position
				result += " " + QString::number(tag.tx).toStdString();
				result += " " + QString::number(tag.ty).toStdString();
				result += " " + QString::number(tag.tz).toStdString();
				result += " " + QString::number(tag.rx).toStdString();
				result += " " + QString::number(tag.ry).toStdString();
				result += " " + QString::number(tag.rz).toStdString() + " ";
				std::cout<<"April position=>"<<result<<std::endl;
			}
		}
		else // No tag
		{
			std::cout<<"Camera: "<<camera<<" no apriltagdetected"<<std::endl;
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error connecting to april: " << e << std::endl;
	}
}

QVec SpecificWorker::updateCameraPosition(string camera, QVec values)
{
	Rot3DOX crx (-values.rx()); //***********RX inverse SIGN************
	Rot3DOY cry (values.ry());
	Rot3DOZ crz (values.rz());
	QMat crZYX = crz * cry * crx;

	RTMat cam = RTMat();
	cam.setR(crZYX);
	cam.setTr(values.x(), values.y(), values.z());
	cam = cam.invert();

	innermodel->getNode<InnerModelTransform>(camera)->setR(cam.getR());
	innermodel->getNode<InnerModelTransform>(camera)->setTr(cam.getTr());
	innermodel->cleanupTables();

	return innermodel->transformS6D("world", camera);		
}

void SpecificWorker::mouseClick(int  event, int  x, int  y)
{
	std::cout<<"*************************"<<std::endl;
	std::cout<<"MOUSE "<< x <<" "<< y<<std::endl;
	std::cout<<"*************************"<<std::endl;

		try{
			RoboCompJointMotor::MotorStateMap hState;
			RoboCompGenericBase::TBaseState bState;
			RoboCompRGBD::DepthSeq points;
			rgbd_proxies[0]->getDepth(points, hState,bState);
			std::cout<<"size"<<points.size()<<" punto: "<<y*640+x<<std::endl;
			
			float depthP = points[y*640+(639-x)];

			QVec vec = QVec::vec3( (x-320) *depthP/535.4  , -(y-240)*depthP/539.2, depthP);
			vec.print("cameraPoint");
			innermodel->transform("world", vec, "camera1").print("mundo");

		}catch(...)
		{
			qDebug()<<"Error connectng to camera: 0";
		}

}

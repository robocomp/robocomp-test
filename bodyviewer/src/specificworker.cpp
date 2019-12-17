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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
    joints_id = JOINTS_ID({
		"Head", 
		"Neck", 
		"ShoulderSpine", 
		"LeftShoulder", 
		"LeftElbow", 
		"LeftWrist", 
		"LeftHand", 
		"RightShoulder", 
		"RightElbow", 
		"RightWrist", 
		"RightHand", 
		"MidSpine", 
		"BaseSpine", 
		"LeftHip", 
		"LeftKnee",
        "LeftFoot",
        "RightHip",
        "RightKnee",
        "RightFoot",
    });

	skeleton = SKELETON_CONNECTIONS({
		{"Head","Neck"},
		{"Neck","ShoulderSpine"},
		{"ShoulderSpine", "LeftShoulder"},
		{"LeftShoulder", "LeftElbow"},
		{"LeftElbow", "LeftWrist"},
		{"LeftWrist", "LeftHand"},
		{"ShoulderSpine", "RightShoulder"},
		{"RightShoulder", "RightElbow"},
		{"RightElbow", "RightWrist"},
		{"RightWrist", "RightHand"},
		{"ShoulderSpine", "MidSpine"},
		{"MidSpine", "BaseSpine"},		
		{"BaseSpine", "LeftHip"},		
		{"LeftHip", "LeftKnee"},
		{"LeftKnee", "LeftFoot"},
		{"BaseSpine", "RightHip"},
		{"RightHip", "RightKnee"},
		{"RightKnee", "RightFoot"}});
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
	scene.setSceneRect(LEFT, BOTTOM, WIDTH, HEIGHT);
	view.scale( 1, -1 );
	view.setScene(&scene);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	resize(view.size());
	view.show();


	const float fx=535.4, fy=539.2, sx=1, sy=1, Ox=320, Oy=240;
	K = QMat::zeros(3,3);
	K(0,0) = fx/sx; K(0,1) = 0.f; 		K(0,2) = Ox;
	K(1,0) = 0; 	 K(1,1) = -fy/sy; 	K(1,2) = Oy;
	K(2,0) = 0;		 K(2,1) = 0;		K(2,2) = 1;
    
	QPen pen(QColor("DarkRed"));
	pen.setWidth(2);


//	scene.addRect(QRectF(0,-4000, 7000, 0),pen);
	scene.addRect(QRectF(10, -400, 700, -10),pen);
	peopleScene.append(QList<QGraphicsEllipseItem*>());
	peopleScene.append(QList<QGraphicsEllipseItem*>());
	peopleScene.append(QList<QGraphicsEllipseItem*>());
	
		for (int j=0;j<5;j++){
			QGraphicsEllipseItem* p = scene.addEllipse(QRectF(-50,-50, 300, 300), QPen(QColor(255, 0, 0)), QBrush(QColor(255, 0, 0)));
			peopleScene[0].append(p);
		}
	for (int j=0;j<5;j++){
			QGraphicsEllipseItem* p = scene.addEllipse(QRectF(-50,-50, 300, 300), QPen(QColor(0, 255, 0)), QBrush(QColor(0, 255, 0)));
			peopleScene[1].append(p);
		}
		for (int j=0;j<5;j++){
			QGraphicsEllipseItem* p = scene.addEllipse(QRectF(-50,-50, 300, 300), QPen(QColor(0, 0, 255)), QBrush(QColor(0, 0, 255)));
			peopleScene[2].append(p);
		}
	

	this->Period = 10;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//	scene.clear();

	static auto beginC = std::chrono::steady_clock::now();
	//draw Complete data
	if (not dataComplete.isEmpty())
	{
		RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB mixedData = dataComplete.get();
        drawBodyOnly(mixedData);
	}
	//draw only skeleton
/*	if (not dataPeople.isEmpty())
	{
		RoboCompHumanTrackerJointsAndRGB::PersonList people = dataPeople.get();
		cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0,0,0));
        drawSkeleton(frame, people);
	}*/
	//Time
	auto endC = std::chrono::steady_clock::now();
//	std::cout << "Compute=>Elapsed = " << std::chrono::duration_cast<std::chrono::milliseconds>(endC-beginC).count() << std::endl;
	beginC = endC;

	
}

void SpecificWorker::drawBodyWithImage(RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB mixedData)
{
	cv::Mat frame(mixedData.rgbImage.height, mixedData.rgbImage.width, CV_8UC3,  &(mixedData.rgbImage.image)[0]);
	cv::cvtColor(frame, frame, CV_BGR2RGB);
	drawBody(frame, mixedData);
}

void SpecificWorker::drawBodyOnly(const RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &mixedData)
{
	//videom
	cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0,0,0));
	drawBody(frame, mixedData);
}

void SpecificWorker::drawBody(const cv::Mat &frame, const RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &mixedData)
{
    //people
	PersonList people = mixedData.persons;
	drawSkeleton(mixedData.cameraID, frame, people);
	updatePersonCamera(mixedData.cameraID, people);
}

void SpecificWorker::updatePersonCamera(int camera, RoboCompHumanTrackerJointsAndRGB::PersonList people)
{
	if (people.size())
	{
		int i=0;
		for(const auto &person : people)
		{
			try{
std::cout<<"camera"<<camera<<"person "<<person.first<<" joint "<<person.second.joints.at("Head")<<std::endl;
				QVec cam = QVec::vec3(-person.second.joints.at("Head")[0], person.second.joints.at("Head")[1], person.second.joints.at("Head")[2]);
				std::string cameraName = "camera" + to_string(camera);
				QVec world = innermodel->transform("world", cam, cameraName.c_str());
world.print("personInWorld");

				//add person ellipse
				
				peopleScene[camera][i]->setPos(world.x(), world.z());
				i++;
				
				
			}catch(...)
			{
				qDebug()<<"No data for midSpine joint, camera:"<<camera;
			}
		}
		while(i<peopleScene[camera].size())
		{
			peopleScene[camera][i]->setPos(0,0);
			i++;
		}
	}
}

void SpecificWorker::drawSkeleton(int camera, const cv::Mat &frame, const RoboCompHumanTrackerJointsAndRGB::PersonList &people)
{
	if (people.size())
	{
		for(const auto &person : people)
		{
			qDebug() << __FUNCTION__ <<"person id = "<<person.first;
			auto p = person.second;
			for (const auto &[first, second] : skeleton)
			{
				try{
					RoboCompHumanTrackerJointsAndRGB::joint j1 = p.joints.at(first);
					RoboCompHumanTrackerJointsAndRGB::joint j2 = p.joints.at(second);
					QVec p13D = QVec::vec3(j1[0], j1[1], j1[2]);
					QVec p23D = QVec::vec3(j2[0], j2[1], j2[2]);
					//convert
					QVec img1 = K * p13D;
					QVec img2 = K * p23D;
					//Draw joint point
					cv::circle(frame,cv::Point(img1[0]/img1[2], img1[1]/img1[2]),10,cv::Scalar(person.first,0,255));
					cv::circle(frame,cv::Point(img2[0]/img2[2], img2[1]/img2[2]),10,cv::Scalar(person.first,0,255));
					
					//Draw bone
					cv::line(frame, cv::Point(img1[0]/img1[2], img1[1]/img1[2]), cv::Point(img2[0]/img2[2], img2[1]/img2[2]), cv::Scalar(person.first,255,0), 2);
				}
				catch(...){}
			}
		}
	}
	cv::imshow(to_string(camera), frame);   
//	cv::imshow(to_string(0), frame);   
}

void SpecificWorker::HumanTrackerJointsAndRGB_newPersonListAndRGB(MixedJointsRGB mixedData)
{
	endS = std::chrono::steady_clock::now();
//    qDebug()<<"Data received, people:"<<mixedData.persons.size()<<"camera"<<mixedData.cameraID;
//	qDebug()<<"Size: "<<mixedData.rgbImage.image.size();
	dataComplete.put(mixedData);
//	dataPeople.put(mixedData.persons);
	//Time
	
//	std::cout << "Storm=>Elapsed = " << std::chrono::duration_cast<std::chrono::milliseconds>(endS-beginS).count() << std::endl;
	beginS = std::chrono::steady_clock::now();
}




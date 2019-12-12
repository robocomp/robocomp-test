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
		"left_elbow", 
		"RightShoulder", 
		"RightElbow", 
		"RightWrist", 
		"RightHand", 
		"MidSpine", 
		"BaseSpine", 
		"LeftHip", 
		"left_ankle", 
		"LeftKnee",
        "LeftFoot",
        "RightHip",
        "RightKnee",
        "RightFoot",
    });

	skeleton = SKELETON_CONNECTIONS({
		{"left_ankle","left_knee"},
		{"left_knee","left_hip"},
		{"right_ankle", "right_knee"},
		{"right_knee", "right_hip"},
		{"left_hip", "right_hip"},
		{"left_shoulder", "left_hip"},
		{"right_shoulder", "right_hip"},
		{"left_shoulder", "right_shoulder"},
		{"left_shoulder", "left_elbow"},
		{"right_shoulder", "right_elbow"},
		{"left_elbow", "left_wrist"},
		{"right_elbow", "right_wrist"},		
		{"left_eye", "right_eye"},		
		{"nose", "left_eye"},
		{"nose", "right_eye"},
		{"left_eye", "left_ear"},
		{"right_eye", "right_ear"},
		{"left_ear", "left_shoulder"},
		{"right_ear", "right_shoulder"}});
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
//	catch(const std::exception &e) { qFatal("Error reading config params"); }



	


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
    
    camera = new Cam(535.4, 539.2, 640/2., 480/2.);
    
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//computeCODE
//QMutexLocker locker(mutex);
//	try
//	{
//		camera_proxy->getYImage(0,img, cState, bState);
//		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
//		searchTags(image_gray);
//	}
//	catch(const Ice::Exception &e)
//	{
//		std::cout << "Error reading from Camera" << e << std::endl;
//	}
}

void SpecificWorker::drawBody(cv::Mat frame, const RoboCompHumanTrackerJointsAndRGB::PersonList &people)
{
	for(auto &person : people)
	{
		qDebug() << __FUNCTION__ <<"person id = "<<person.first;
        auto p = person.second;
		for (auto &[first, second] : skeleton)
		{
            try{
                RoboCompHumanTrackerJointsAndRGB::joint j1 = p.joints.at(first);
                RoboCompHumanTrackerJointsAndRGB::joint j2 = p.joints.at(second);
                QVec p13D = QVec::vec3(j1[0],j1[1], j1[2]);
                QVec p23D = QVec::vec3(j2[0],j2[1], j2[2]);
                //convert
                QVec img1 = camera->getRayHomogeneous(p13D);
                QVec img2 = camera->getRayHomogeneous(p23D);
                //Draw joint point
                cv::circle(frame,cv::Point(img1[0], img1[1]),10,cv::Scalar(0,0,255));
                cv::circle(frame,cv::Point(img2[0], img2[1]),10,cv::Scalar(0,0,255));
                
                //Draw bone
                //cv::line(frame, cv::Point(j1->x, j1->y), cv::Point(j2->x, j2->y), cv::Scalar(0,255,0), 2);
             }
             catch(...){}
		}
	}
	cv::imshow("People", frame);
	
}


void SpecificWorker::HumanTrackerJointsAndRGB_newPersonListAndRGB(MixedJointsRGB mixedData)
{
    qDebug()<<"Data received, people:"<<mixedData.persons.size();
    //video
	cv::Mat frame(mixedData.rgbImage.height, mixedData.rgbImage.width, CV_8UC3,  &(mixedData.rgbImage.image)[0]);
	cv::cvtColor(frame, frame, CV_BGR2RGB);
    cv::imshow("People", frame);
    
    //people
    PersonList people = mixedData.persons;
    if (people.size())
        drawBody(frame, people);
   
    
}




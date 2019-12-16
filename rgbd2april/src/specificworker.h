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
#include "opencv2/opencv.hpp"
#include <cppitertools/itertools.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void computeApril(RoboCompRGBD::ColorSeq color, int camera);



	QVec updateCameraPosition(string camera, QVec values);
	void mouseClick(int  event, int  x, int  y);

public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innermodel;
	std::vector<RGBDPrx> rgbd_proxies;
	std::vector<QVec> cameras;
};

//OpenCV Mouse callback
static void mouse_callback(int event, int x, int y, int, void* userdata)
{
	if  ( event == cv::EVENT_LBUTTONDOWN )
	{
		// Check for null pointer in userdata and handle the error
		SpecificWorker* worker = reinterpret_cast<SpecificWorker*>(userdata);
		worker->mouseClick(event, x, y);
		std::cout<<"mouse_callback";
	}
}


#endif

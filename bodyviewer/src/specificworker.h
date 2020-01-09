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
#include <opencv2/opencv.hpp>
#include "doublebuffer.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>
#include <QGraphicsEllipseItem>


using SKELETON_CONNECTIONS = std::vector<std::tuple<std::string, std::string>>;
using JOINTS_ID = std::vector<std::string>;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void HumanTrackerJointsAndRGB_newPersonListAndRGB( RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB mixedData);
	void drawBodyWithImage(RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB mixedData);
    void drawBodyOnly(const RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &mixedData);
	void drawBody(const cv::Mat &frame, const RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &mixedData);
    void drawSkeleton(int camera, const cv::Mat &frame, const RoboCompHumanTrackerJointsAndRGB::PersonList &people);
	void updatePersonCamera(int camera, RoboCompHumanTrackerJointsAndRGB::PersonList people);

public slots:
	void compute();
	void initialize(int period);
private:
	const float LEFT = 0, BOTTOM = -4000, WIDTH = 7000, HEIGHT = 4000;
	QGraphicsScene scene;
	QGraphicsView view;
	QList<QList<QGraphicsEllipseItem*>> peopleScene;

	std::shared_ptr<InnerModel> innermodel;
	DoubleBuffer<RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB, RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB> dataComplete;
	DoubleBuffer<RoboCompHumanTrackerJointsAndRGB::PersonList, RoboCompHumanTrackerJointsAndRGB::PersonList> dataPeople;
    QMat K;
    SKELETON_CONNECTIONS skeleton;
	JOINTS_ID joints_id;

	std::chrono::steady_clock::time_point beginS, endS;
};

#endif

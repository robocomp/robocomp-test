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
#include <doublebuffer/DoubleBuffer.h>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(TuplePrx tprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		void IMUPub_publish(RoboCompIMU::DataImu imu);

	public slots:
		void compute();
		void initialize(int period);

	private:
		//constants
		const float LEFT = -790, BOTTOM = 0, WIDTH = 5960, HEIGHT = 9700;
		const float ROBOT_LENGTH = 400;
		float initX, initZ;
		float initialAngle = -9999, angleOffset = -9999;

		std::shared_ptr<InnerModel> innerModel;
		QGraphicsScene scene;
		QGraphicsView view;
		QGraphicsPolygonItem *robot, *tail, *robot_stable;
		std::vector<QGraphicsItem*> boxes;
		float yaw_class = 0;
		QGraphicsEllipseItem *p_circle;

		void initializeWorld();
		float degreesToRadians(const float angle_);

	protected:
		void mousePressEvent(QMouseEvent *event) override;
		void wheelEvent(QWheelEvent *event) override;
		void resizeEvent(QResizeEvent *event) override;

};

#endif

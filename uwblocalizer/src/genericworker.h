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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>


#include <CommonBehavior.h>

#include <DifferentialRobot.h>
#include <GenericBase.h>
#include <FullPoseEstimation.h>
#include <IMUPub.h>
#include <IMU.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompIMU;
using namespace RoboCompIMUPub;
using namespace RoboCompGenericBase;
using namespace RoboCompDifferentialRobot;
using namespace RoboCompFullPoseEstimation;

using TuplePrx = std::tuple<RoboCompDifferentialRobot::DifferentialRobotPrxPtr>;


class GenericWorker :
public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	DifferentialRobotPrxPtr differentialrobot_proxy;

	virtual FullPose FullPoseEstimation_getFullPose() = 0;
	virtual void GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void GenericBase_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void IMUPub_publish(RoboCompIMU::DataImu imu) = 0;

protected:
	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
signals:
	void kill();
};

#endif

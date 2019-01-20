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

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>

#include <CommonBehavior.h>

#include <TrajectoryRobot2D.h>
#include <Logger.h>
#include <OmniRobot.h>
#include <GenericBase.h>
#include <Laser.h>
#include <GenericBase.h>
#include <RCISMousePicker.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompLaser;
using namespace RoboCompTrajectoryRobot2D;
using namespace RoboCompOmniRobot;
using namespace RoboCompGenericBase;
using namespace RoboCompLogger;
using namespace RoboCompRCISMousePicker;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


class GenericWorker :
#ifdef USE_QTGUI
	public QWidget, public Ui_guiDlg
#else
	public QObject
 #endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	OmniRobotPrx omnirobot_proxy;
	LaserPrx laser_proxy;
	LoggerPrx logger_proxy;

	virtual NavState TrajectoryRobot2D_getState() = 0;
	virtual float TrajectoryRobot2D_goBackwards(const TargetPose &target) = 0;
	virtual void TrajectoryRobot2D_stop() = 0;
	virtual void TrajectoryRobot2D_setHumanSpace(const PolyLineList &polyList) = 0;
	virtual float TrajectoryRobot2D_goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold) = 0;
	virtual float TrajectoryRobot2D_changeTarget(const TargetPose &target) = 0;
	virtual float TrajectoryRobot2D_go(const TargetPose &target) = 0;
	virtual void TrajectoryRobot2D_mapBasedTarget(const NavigationParameterMap &parameters) = 0;
	virtual void RCISMousePicker_setPick(const Pick &myPick) = 0;

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

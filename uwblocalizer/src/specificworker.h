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
#include <QtSerialPort/QSerialPort>
#include <iostream>
#include <doublebuffer/DoubleBuffer.h>

// #include <QGraphicsScene>
// #include <QGraphicsView>
// #include <QGraphicsEllipseItem>
// #include <QGraphicsLineItem>
// #include <QGraphicsRectItem>
// #include <QGraphicsPolygonItem>


//kalman
#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
using namespace KalmanExamples;
typedef float T;
// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;
#include <random>
#include <chrono>

class MyConverter : public Converter<std::vector<int>, FullPose>
{
public:
    bool ItoO(const std::vector<int> &iTypeData, RoboCompFullPoseEstimation::FullPose &oTypeData)
    {
        if (iTypeData.size() == 3)
		{
			oTypeData.x = iTypeData[0];
			oTypeData.y = iTypeData[1];
			oTypeData.z = iTypeData[2];
			oTypeData.rz = 0.;
			oTypeData.ry = 0.;
			oTypeData.rz = 0.;
            return true;
        }
        return false;
    }
    bool OtoI(const RoboCompFullPoseEstimation::FullPose &oTypeData, std::vector<int> &iTypeData)
    {
        return false;
    }
    bool clear(RoboCompFullPoseEstimation::FullPose &oTypeData)
    {
	return true;
    }
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    void IMUPub_publish(RoboCompIMU::DataImu imu);
    void GenericBase_getBaseState(RoboCompGenericBase::TBaseState &state);
	void GenericBase_getBasePose(int &x, int &z, float &alpha);
	FullPose FullPoseEstimation_getFullPose();

public slots:
	void compute();
	void initialize(int period);
	std::vector<int> readData(QSerialPort &serial);

private:
	std::vector<int>  posL{0,0,0}, posR{0,0,0};
	MyConverter myconverter;
	DoubleBuffer<std::vector<int>, RoboCompFullPoseEstimation::FullPose, MyConverter> dbuffer;

	//kalman
	State x;
	Control u;
	SystemModel sys;
	PositionModel pm;
	OrientationModel om;
	Kalman::ExtendedKalmanFilter<State> predictor;
	Kalman::ExtendedKalmanFilter<State> ekf;
	Kalman::UnscentedKalmanFilter<State> ukf;

	// Random number generation (for noise simulation)
    std::default_random_engine generator;
	std::normal_distribution<float> noise;
	//std::uniform_int_distribution<> noise;
	
	void init_kalman();
	State x_ekf;

	//constants
	const float LEFT = -790, BOTTOM = 0, WIDTH = 5960, HEIGHT = 9700;
	const float ROBOT_LENGTH = 400;

	std::vector<State> sensor_data;

	RoboCompGenericBase::TBaseState bState;
	InnerModel *innerModel;
    std::vector<QPointF> qPosL;
	std::vector<QPointF> qPosR;
	float xMedL = 0;
	float yMedL = 0;
	float xMedR = 0;
	float yMedR = 0;
	float initialAngle, correctedAngle;

	QSerialPort serial_left, serial_right; 
// 	QGraphicsScene scene;
// 	QGraphicsView view;
// 	QGraphicsPolygonItem *robot, *tail, *robot_stable;
// 	std::vector<QGraphicsItem*> boxes;
// 	float yaw_class = 0;
// 	QGraphicsEllipseItem *p_circle;
// 
// 	QTimer sensor_timer;

  	void initializeWorld();
    
	template <typename IntegerType>
	IntegerType bitsToInt( const unsigned char* bits, uint init, bool little_endian = true )
	{
		IntegerType result = 0;
		if (little_endian)
			for (int n = sizeof( result ); n >= 0; n--)
			result = (result << 8) + bits[ init + n ];
		else
			for (unsigned n = 0; n < sizeof( result ); n++)
			result = (result << 8) + bits[ n ];
		return result;
	}
    //void compute_initial_pose(int ntimes);
	//void compute_pose();
	float degreesToRadians(const float angle_);
    
protected:
// 		void mousePressEvent(QMouseEvent *event) override;
// 		void wheelEvent(QWheelEvent *event) override;
// 		void resizeEvent(QResizeEvent *event) override;
};

#endif

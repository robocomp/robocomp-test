#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, time

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

preStr = "-I"+ROBOCOMP+"/interfaces/ -I/opt/robocomp/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
import RoboCompCommonBehavior
import RoboCompTrajectoryRobot2D


class PointWidget(QtGui.QWidget):
	pressed = QtCore.Signal(int)

	def __init__(self, prnt, x, z, a, ident):
		super(PointWidget, self).__init__()

		self.x = x
		self.z = z
		self.a = a
		self.ident = ident
		self.layout = QtGui.QVBoxLayout()

		lo = QtGui.QHBoxLayout()
		x = QtGui.QLabel("x"+str(ident))
		lo.addWidget(x)
		self.xLabel = QtGui.QLabel(str(self.x))
		lo.addWidget(self.xLabel)
		self.layout.addLayout(lo)

		lo = QtGui.QHBoxLayout()
		z = QtGui.QLabel("z"+str(ident))
		lo.addWidget(z)
		self.zLabel = QtGui.QLabel(str(self.z))
		lo.addWidget(self.zLabel)
		self.layout.addLayout(lo)

		lo = QtGui.QHBoxLayout()
		a = QtGui.QLabel("angle")
		lo.addWidget(a)
		self.aLabel = QtGui.QLabel(str(self.a))
		lo.addWidget(self.aLabel)
		self.layout.addLayout(lo)

		self.checkbox = QtGui.QCheckBox("current")
		self.checkbox.setEnabled(False)
		self.layout.addWidget(self.checkbox)

		self.button = QtGui.QPushButton("send")
		QtCore.QObject.connect(self.button, QtCore.SIGNAL ('clicked()'), self.pressedSlot)
		self.layout.addWidget(self.button)

		self.stopButton = QtGui.QPushButton("stop")
		self.layout.addWidget(self.stopButton)

		self.setLayout(self.layout)
		self.show()

	@QtCore.Slot()
	def pressedSlot(self):
		#print 'PointWidget::pressedSlot', self.ident
		self.pressed.emit(self.ident)
		
	
class SpecificWorker(GenericWorker):
	currentTarget = -1
	points = []
	pointWidgets = []
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.vlayout = QtGui.QVBoxLayout()
		self.layout = QtGui.QHBoxLayout()
		self.vlayout.addLayout(self.layout)
		self.setLayout(self.vlayout)

	def setParams(self, params):
		try:
			npoints = int(params["Points"])
		except:
			print 'Error getting the number of points from config file'
		
		for npoint in xrange( npoints ):
			points_str = params["Point"+str(npoint)].split(',')
			x = points_str[0]
			z = points_str[1]
			a = None
			if len(points_str) > 2:
				a = points_str[2]
				point = [ float(x), float(z), float(a) ]
			else:
				point = [ float(x), float(z), None ]
			self.points.append(point)		
			print point
			
		for npoint in xrange( len(self.points) ):
			pw = PointWidget(prnt=None, x=self.points[npoint][0], z=self.points[npoint][1], a=self.points[npoint][2], ident=npoint)
			QtCore.QObject.connect(pw, QtCore.SIGNAL('pressed(int)'), self.pressedSlot)
			pw.stopButton.clicked.connect( self.stopRobotSlot )
			self.layout.addWidget(pw)
			self.pointWidgets.append(pw)

		self.edit = QtGui.QTextEdit()
		self.vlayout.addWidget(self.edit)

		self.Period = 200

	@QtCore.Slot()
	def stopRobotSlot(self):
		try:
			self.trajectoryrobot2d_proxy.stop()
		except Ice.Exception as e:
			print e
		

	@QtCore.Slot(int)
	def pressedSlot(self, ident):
		print 'SpecificWorker::pressedSlot', ident
		if self.currentTarget > -1:
			self.pointWidgets[self.currentTarget].checkbox.setChecked(False)
		self.currentTarget = ident
		self.pointWidgets[self.currentTarget].checkbox.setChecked(True)
		target = RoboCompTrajectoryRobot2D.TargetPose()
		target.x = self.points[self.currentTarget][0]
		target.y = 0
		target.z = self.points[self.currentTarget][1]
		target.rx = 0
		
		if self.points[self.currentTarget][2] == None:
			target.ry = 0
			target.doRotation = False
		else:
			target.ry = float(self.points[self.currentTarget][2])
			target.doRotation = True
		target.rz = 0
		try:
			self.trajectoryrobot2d_proxy.go(target)
		except Ice.Exception as e:
			print e

	@QtCore.Slot()
	def compute(self):
		try:
			s = self.trajectoryrobot2d_proxy.getState()
			print "--------- NavState -----------"
			print "	STATE", s.state
			print "	pose:" , "%.2f" % s.x, "%.2f" % s.z, "%.2f" % s.ang
			print "	vel pose:", s.advV, s.rotV
			print "	dist to target", s.distanceToTarget
			print "	elapsedTime", s.elapsedTime
			print "	ETA", s.estimatedTime
			print " planning time", s.planningTime
			self.edit.append("STATE: " + s.state 
										+ "	X: " + "%d mm" % s.x + "  Z: %d mm" % s.z + "  A: %.2f r" % s.ang 
										+ "	vA: " + "%d mm/s" % s.advV + "vR %.2f r/s" % s.rotV
										+ "	D2target: " + "%d mm" % s.distanceToTarget 
										+ "	ET: " + "%d ms" % s.elapsedTime 
										+ "	ETA: " + "%d ms" % s.estimatedTime
										+ " PT: " + "%d ms" % s.planningTime )
			
		except Ice.Exception as e:
			print e
		
################
## SUBSCRIPTIONS
################

	#
	# setPick
	#
	def setPick(self, myPick):
		#
		print "from RCIS", myPick
		target = RoboCompTrajectoryRobot2D.TargetPose()
		target.x = myPick.x
		target.y = 0
		target.z = -myPick.z
		target.doRotation = False
		try:
			self.trajectoryrobot2d_proxy.go(target)
		except Ice.Exception as e:
			print e


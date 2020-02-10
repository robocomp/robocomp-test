#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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

from genericworker import *

import pickle, os
import numpy as np
import cv2

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.rfile = open("prueba", 'rb')
		self.im = TImage()
		self.dep = TDepth()

	def __del__(self):
		print('SpecificWorker destructor')
		if self.rfile:
			self.rfile.close()

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')

		# Loop file
		if self.rfile.tell() == os.stat("prueba").st_size:
			self.rfile.seek(0, 0)

		# read
		self.im = pickle.load(self.rfile)
		self.dep = pickle.load(self.rfile)

#		color = np.frombuffer(self.im.image, np.uint8).reshape(self.im.height, self.im.width, self.im.depth)
#		cv2.imshow("Pub_frame", color)


		return True

# =============== Methods for Component Implements ==================
# ===================================================================

	#
	# getAll
	#
	def CameraRGBDSimple_getAll(self):
		return self.im, self.dep

	#
	# getDepth
	#
	def CameraRGBDSimple_getDepth(self):
		return self.dep

	#
	# getImage
	#
	def CameraRGBDSimple_getImage(self):
		return self.im

# ===================================================================
# ===================================================================


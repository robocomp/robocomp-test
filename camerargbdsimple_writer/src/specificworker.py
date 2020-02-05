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

import time, pickle
import numpy as np
import cv2
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.contFPS = 0
		self.initialize()
		self.start = time.time()

	def __del__(self):
		if self.wfile:
			self.wfile.close()

		print('SpecificWorker destructor')

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		return True


	def initialize(self):
		self.wfile = open( "prueba", 'wb')

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		return

		try:
			color_, depth_ = self.camerargbdsimple_proxy.getAll()
			if (len(color_.image) == 0) or (len(depth_.depth) == 0):
				print('Error retrieving images!')
			else:
				color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
				cv2.imshow("RPC_frame", color)
		except:
			print("Exception reading images: ", sys.exc_info()[0])


		return True


	#
	# pushRGBD
	#
	def CameraRGBDSimplePub_pushRGBD(self, im, dep):
		print("received data")

		color = np.frombuffer(im.image, np.uint8).reshape(im.height, im.width, im.depth)
		cv2.imshow("Pub_frame", color)

#		pickle.dump(im, self.wfile)
#		pickle.dump(dep, self.wfile)

		if time.time() - self.start > 1:
			print("FPS:", self.contFPS)
			self.start = time.time()
			self.contFPS = 0
		self.contFPS += 1


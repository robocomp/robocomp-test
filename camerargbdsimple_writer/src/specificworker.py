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
		self.Period = 100
		self.timer.start(self.Period)
		self.contFPS = 0
		self.wfile = None

	def __del__(self):
		if self.wfile:
			self.wfile.close()

		print('SpecificWorker destructor')

	def setParams(self, params):
		self.capture_rpc = "true" in params["capture_rpc"]
		self.capture_pub = "true" in params["capture_pub"]
		self.output_file = params["output_file"]
		self.initialize()
		return True


	def initialize(self):
		self.wfile = open( self.output_file, 'wb')
		self.start = time.time()

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')

		if self.capture_rpc:
			try:
				color_, depth_ = self.camerargbdsimple_proxy.getAll()
				if (len(color_.image) == 0) or (len(depth_.depth) == 0):
					print('Error retrieving images!')
				else:
#					color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
#					cv2.imshow("RPC_frame", color)
					pickle.dump(im, self.wfile)
					pickle.dump(dep, self.wfile)
					if time.time() - self.start > 1:
						print("FPS:", self.contFPS)
						self.start = time.time()
						self.contFPS = 0
					self.contFPS += 1
			except:
				print("Exception reading images: ", sys.exc_info()[0])

		return True


	#
	# pushRGBD
	#
	def CameraRGBDSimplePub_pushRGBD(self, im, dep):
		if self.capture_pub:
	#		print("received data")

	#		color = np.frombuffer(im.image, np.uint8).reshape(im.height, im.width, im.depth)
			try:
	#			cv2.imshow("Pub_frame", color)
				pass
			except :
				print("Exception reading images: ", sys.exc_info()[0])

			pickle.dump(im, self.wfile)
			pickle.dump(dep, self.wfile)

			if time.time() - self.start > 1:
				print("FPS:", self.contFPS)
				self.start = time.time()
				self.contFPS = 0
			self.contFPS += 1


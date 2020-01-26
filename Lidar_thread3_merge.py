
# -*- coding: utf-8 -*-

import serial, threading, math, time, pygame, copy, numpy, itertools, binascii, struct
from sys import exit
from pygame.locals import *
from rdp import rdp

data_list_buffer = [] # shared item between window thread and lidar thread. Lidar stores data in the list and window thread uses it as initial data
state_mode_IMU = []
state_mode = []
thread_queue = {}
lidar_angle_buffer = [] # shared item between window thread and IMU thread. Length of the buffer is defined by LIDAR_BUFFER_LEN

data_buffer_lock = threading.Lock()
IMU_buffer_lock = threading.Lock()
state_lock = threading.Lock()
state_lock_IMU = threading.Lock()

DOUGLAS_EPSILON = 15
ANGLE_CHANGE_TOL = 8
ANGLE_TOL = 40
ANGLE_TOL2 = 20
FOOTHOLD_MIN_LEN = 50
FOOTHOLD_MAX_LEN = 500
OPTIMUM_STEP_LEN = 700
STEP_OFFSET = 200
MIN_PTS_DENSITY = 0.008
STEPS_PREDICT_NUM = 5
PAUSE = False
LIDAR_OFFSET = 4.4
LIDAR_BUFFER_LEN = 50
DRAWBOLN = True

class serial_read_error(Exception):
	pass
class serial_read_time_out(Exception):
	pass

class radar_serial_thread(serial.Serial, threading.Thread):
	global data_list_buffer, state_mode

	def __init__(self, threadID, name, S, B = 115200):
		threading.Thread.__init__(self)
		thread_queue[threadID] = self
		# serial.Serial.__init__(self, serial_port, Baud)

		self.serial_port = S
		self.baud = B
		self.threadID = threadID
		self.name = name
		self.start_time = 0
		self.points_data = []
		self.state_map = {\
							b'\x01':"雷达CCD故障",\
							b'\x02':"电机转动速度不稳定:::检查电机",\
							b'\x03':"雷达配置值丢失",\
							b'\x04':"激光发射管故障"\
						  }
		# init the serial

		try:
			serial.Serial.__init__(self, self.serial_port, self.baud)
		except serial.serialutil.SerialException:
			state_lock.acquire()
			try:
				state_mode.clear()
				state_mode.append("串口设备  "+self.serial_port+"  不存在")
			finally:
				state_lock.release()
				raise serial.serialutil.SerialException

	def restart(self):
		try:
			serial.Serial.__init__(self, self.serial_port, self.baud)
		except serial.serialutil.SerialException:
			state_lock.acquire()
			try:
				state_mode.clear()
				state_mode.append("串口设备  "+self.serial_port+"  不存在")
			finally:
				state_lock.release()
				raise serial.serialutil.SerialException

	def Head_check(self):
		# IO intensive
		'''阻塞(time_out = 5s)，直到检测到‘\xAA’；冗余检测；读取数据帧长度值和命令符；返回（isData，有效数据长度）'''
		try:
			self.start_time = time.time()
			#start code check
			# print("check start")
			# while True:
			#     print(self.read())
			while self.read() != b'\xAA':
				if (time.time() - self.start_time) > 5:
					raise serial_read_time_out

			length = self.twobyte2int()-3
			# print("1")

			#address and type code check
			if self.read(2) != b'\x00\x61':
				raise serial_read_error
			length -= 2

			#command code read
			r = self.read()
			length -= 1
			# print("2")

			#length check
			length_buffer = self.twobyte2int()
			length -= 2
			if length != length_buffer:
				raise serial_read_error

			#command code check
			if r == b'\xA9':
				isData = True
			elif r == b'\xAB':
				isData = False
			return (isData, length)
		except serial_read_error:
			print("Serial data don't accord with the protocol ...... Data ignored")
		except serial_read_time_out:
			print("Head_check time out, Please comfirm the device is legal")
			raise serial_read_time_out

	def twobyte2int(self):
		# IO intensive
		buffer = self.read()[0]
		int_number = buffer*256 + self.read()[0]
		return int_number

	def data_decode(self, L):# 解码运算一帧数据
		# IO intensive
		state_lock.acquire()
		try:
			if len(state_mode)>0:
				state_mode.clear()
		finally:
			state_lock.release()
		angle = self.twobyte2int()*0.01
		self.points_data = []
		L -= 2
		N = int(L/2)
		# print("N is ", N)
		for i in range(N):
			distance = self.twobyte2int()*0.25
			if distance != 0:
				self.points_data.append((angle, distance, time.time()))
			angle += 22.5/N
			angle = round(angle,2)
		self.read(2)

	def state_decode(self, L):
		'''
		设备故障代码
		0x00:   无故障
		0x01:   CCD故障
		0x02:   雷达转速不稳
		0x03:   雷达配置值丢失
		0x04:   激光管故障
		'''
		error_code = self.read()
		print(error_code)
		print(state_mode)
		self.read(2)
		state_lock.acquire()
		try:
			if   error_code == b'\x00':
				state_mode.clear()
			else:
				if len(state_mode) == 0:
					state_mode.append(self.state_map[error_code])
		finally:
			state_lock.release()

	def first_angle_print(self):
		# test method
		buffer_h = self.Head_check()
		if buffer_h:
			isData, L = buffer_h
			if isData:
				print(self.twobyte2int()*0.01)
				L -= 2
				N = int(L/2)
				self.read(N)
			else:
				print("error")

	def run(self):
		while True:
			try:
				if (not state_mode) or state_mode[0] != "串口设备  "+self.serial_port+"  不存在":
					buffer_h = self.Head_check()
					if buffer_h:
						# print('\nhead check...')
						isData, Length = buffer_h
						if isData:
							# print('Is data')
							# print(self.in_waiting)
							self.data_decode(Length)
							data_buffer_lock.acquire()
							try:
								data_list_buffer.append(self.points_data)#当前帧数据加入线程共享数据队列
							finally:
								data_buffer_lock.release()
						else:
							self.state_decode(Length)
			except serial_read_time_out:
				if len(state_mode) == 0:
					state_lock.acquire()
					try:
						state_mode.append("寻找帧开头超时:::请确认已正确连接合法设备并设置串口")
					finally:
						state_lock.release()
			except serial.serialutil.SerialException:
				state_lock.acquire()
				try:
					state_mode.clear()
					state_mode.append("串口设备  "+self.serial_port+"  丢失  按下command+R重连")
				finally:
					state_lock.release()

class window_thread(threading.Thread):
	'''
	Thread that conducts data analysis and recognition algorithm. This thread also generates a visualisation window to plot the data. 
	'''
	global data_list_buffer, state_mode

	def __init__(self, threadID, name, drawBoln):
		threading.Thread.__init__(self)
		thread_queue[threadID] = self
		self.threadID = threadID
		self.name = name
		self.drawBoln = drawBoln
		self.polar_points = []
		self.polar_points_new = []
		self.K = math.pi/180
		self.points_analyse = []
		self.points_reduced = []
		self.steps_coord = {}
		self.points_reduced_index = []
		self.seg_angle_list = []
		self.redundant_cluster = []
		self.data_select = []
		self.hidden_corners = []
		self.lidar_angle = 0
		self.footholds = []

		if drawBoln:

			self.SCREEN_SIZE = (2560, 1400)
			# pygame window init
			pygame.init()
			self.screen = pygame.display.set_mode(self.SCREEN_SIZE, RESIZABLE, 32)
			pygame.display.set_caption("Radar Points")

			# 载入图片
			self.point = pygame.image.load('point.png').convert_alpha()
			self.warning_image = pygame.image.load('warning.png').convert_alpha()

			# 字体设置
			self.font = pygame.font.Font("PingFang.ttf", 70)

			# point图片长款补偿
			self.W,self.H = self.point.get_width()/2 , self.point.get_height()/2
			self.X,self.Y = self.SCREEN_SIZE[0]/10 , self.SCREEN_SIZE[0]/10
			self.k = self.SCREEN_SIZE[1]/2000
			self.Shift_T = 50    #文本Y轴偏移
			self.Shift_W = 200   #标志Y轴偏移

			self.D = 2

	def projection(self):
		# CPU intensive
		'''
		takes one set of data and and calculate the average time of this set. The imu data is selected based on this calculated time. 
		The data is then offseted by this IMU angle selected and loaded into self.points_analyse
		Calls the function self.analyse
		'''
		self.points_analyse.clear()
		average_time = (self.polar_points[0][2] + self.polar_points[-1][2])/2
		self.get_lidar_angle(average_time)
		for n in self.polar_points:
			adjusted_angle = n[0] + self.lidar_angle
			if adjusted_angle > 360:
				adjusted_angle -= 360
			elif adjusted_angle < 0:
				adjusted_angle += 360
			self.polar_points_new.append((adjusted_angle, n[1]))	
		self.polar_points_new = sorted(self.polar_points_new)
		for m in self.polar_points_new:
			if m[0] <= 360 and m[0] >= 270:
				xx, yy = (-(m[1] * math.sin(self.K * m[0])), (m[1] * math.cos(self.K * m[0])))
				self.points_analyse.append((xx,-yy))
		if self.points_analyse:
			self.Analyse()

	def get_lidar_angle(self, lidar_time):
		IMU_buffer_lock.acquire()
		try:
			if lidar_angle_buffer:
				self.lidar_angle = min(lidar_angle_buffer, key=lambda x:abs(x[0]-lidar_time))[1]
			else:
				print('No IMU Data')
		finally:
			IMU_buffer_lock.release()

	def Analyse(self):
		remove_index = []
		seg_angle_list = []
		redundant_data = []
		self.footholds.clear()
		changed_flag = False
		outlier_flag = False
		self.redundant_cluster.clear()
		self.points_analyse.reverse()
		self.data_select = self.points_analyse
		n = len(self.points_analyse)
		mask = rdp(self.points_analyse, epsilon = DOUGLAS_EPSILON, return_mask = True) # RDP algorithm that simplifies points
		self.points_reduced_index = numpy.ndarray.tolist(numpy.where(mask)[0]) # a list that records the index of each points_reduced selected in the original long data list
		pts_reduced = list(itertools.compress(self.points_analyse, mask))
		if pts_reduced and pts_reduced[0]:
			ptList_reduced_polar= self.coord_to_polar(pts_reduced)
			pts_reduced = [y for (x,y) in sorted(zip(list(zip(*ptList_reduced_polar))[0], pts_reduced))]
			
			#Discard point in pts_reduced if no points in between a segment
			for p in range(len(pts_reduced) - 1):
				index_change = self.points_reduced_index[p+1] - self.points_reduced_index[p]
				if outlier_flag:
					if index_change == 1:
						remove_index.append(p)
						changed_flag = True
					else:
						outlier_flag = False
				elif index_change == 1:
					outlier_flag = True
				if not changed_flag:
					deltax = pts_reduced[p+1][0] - pts_reduced[p][0]
					deltay = pts_reduced[p+1][1] - pts_reduced[p][1]
					seg_angle = math.atan2(deltay, deltax)*180/math.pi
					if seg_angle < -180:
						seg_angle += 360.0
					elif seg_angle > 180:
						seg_angle -= 360
					seg_angle_list.append(seg_angle)

			#if not enough angle change, discard the midpoint
			for q in range(len(seg_angle_list) - 1):
				if abs(seg_angle_list[q+1] - seg_angle_list[q]) <= ANGLE_CHANGE_TOL:
					if self.points_reduced_index[q+2] - self.points_reduced_index[q+1] > 1:
						remove_index.append(q+1)
						changed_flag = True
			#remove points and remake seg_angle_list
			if changed_flag:
				remove_index = sorted(list(set(remove_index)))
				remove_index.reverse()
				for item in remove_index:
					self.points_reduced_index.pop(item)
					pts_reduced.pop(item)
				seg_angle_list.clear()
				for t in range(len(pts_reduced) - 1):
					deltax = pts_reduced[t+1][0] - pts_reduced[t][0]
					deltay = pts_reduced[t+1][1] - pts_reduced[t][1]
					seg_angle = math.atan2(deltay, deltax)*180/math.pi
					if seg_angle < -180:
						seg_angle += 360.0
					elif seg_angle > 180:
						seg_angle -= 360
					seg_angle_list.append(seg_angle)

			#check for the requirements of a foothold segment and generate the footholds list 
			for index in range(len(pts_reduced) - 1):
				deltadist = self.calc_distance(pts_reduced[index+1], pts_reduced[index])
				pts_in_btw = self.points_reduced_index[index+1] - self.points_reduced_index[index] 
				pts_density = pts_in_btw/deltadist
				if pts_in_btw > 1:
					if pts_density > MIN_PTS_DENSITY:
						if abs(seg_angle_list[index]) < ANGLE_TOL:
							if self.footholds:
								distance = self.calc_distance(pts_reduced[self.footholds[-1][0]+1], pts_reduced[index])
								if distance < FOOTHOLD_MAX_LEN:
									self.footholds.append([index, deltadist, seg_angle_list[index]])
							else:
								self.footholds.append([index, deltadist, seg_angle_list[index]])
			#mark the first point of the remaining data that doesn't contain a foothold as redundant_data_start
			if self.footholds:
				if self.footholds[-1][0] == len(self.points_reduced_index) - 2:
					redundant_data_start = None
				else:
					redundant_data_start = self.points_reduced_index[self.footholds[-1][0]+1] + 1
					pts_reduced = pts_reduced[0:self.footholds[-1][0]+2]
					self.points_reduced_index = self.points_reduced_index[0:self.footholds[-1][0]+2]
					self.data_select = self.points_analyse[0:redundant_data_start]
			else:
				redundant_data_start = None
			if redundant_data_start:
				self.redundant_cluster = self.points_analyse[redundant_data_start:]
			self.seg_angle_list = seg_angle_list
			self.points_reduced = pts_reduced
			del pts_reduced, seg_angle_list, redundant_data
			self.mode_recognition()

	def coord_to_polar(self, ptList):
		ptList_polar = []
		for pt in ptList:
			new_angle = math.atan2(pt[1], pt[0]) * 180/math.pi
			dist = math.sqrt(pt[0]**2 + pt[1]**2)
			ptList_polar.append((new_angle, dist))
		return ptList_polar

	def make_list_from_tuples(self, *args):
		empty_list = []
		for item in args:
			empty_list.append(item)
		return empty_list

	def mode_recognition(self):
		current_foothold_num = 0
		self.hidden_corners = []
		self.steps_coord = {}
		steps_coord = {0: [0, self.points_reduced[0]]}
		step_coord = self.points_reduced[0]
		j = 0
		if len(self.footholds) > 0:
			while j <= STEPS_PREDICT_NUM:
				if len(self.footholds) > current_foothold_num:
					#if one foothold segment is longer than the OPTIMUM_STEP_LEN, then choose the point on the current segment
					if (self.calc_distance(self.points_reduced[self.footholds[current_foothold_num][0] + 1], step_coord) > OPTIMUM_STEP_LEN):
						step_coord = self.find_point(step_coord, OPTIMUM_STEP_LEN, self.footholds[current_foothold_num][2], self.footholds[current_foothold_num][0])
						steps_coord[j+1] = [self.footholds[current_foothold_num][0], step_coord]
						j += 1
					else:
						current_foothold_num += 1
						if len(self.footholds) > current_foothold_num:
							if self.points_reduced[self.footholds[current_foothold_num][0]][1] - self.points_reduced[self.footholds[current_foothold_num-1][0]][1] >= 0:
								#Ascent case
								if self.footholds[current_foothold_num][1] > FOOTHOLD_MIN_LEN:
									step_coord = self.find_point(self.points_reduced[self.footholds[current_foothold_num][0]], STEP_OFFSET, self.footholds[current_foothold_num][2], self.footholds[current_foothold_num][0])
									if self.calc_distance(step_coord, steps_coord[j][1]) > FOOTHOLD_MIN_LEN:
										steps_coord[j+1] = [self.footholds[current_foothold_num][0], step_coord]
										j += 1
							else:
								#Descent case
								perpendicular_angle = self.seg_angle_list[self.footholds[current_foothold_num][0]-2] - 90
								if abs(self.seg_angle_list[self.footholds[current_foothold_num][0]] - self.seg_angle_list[self.footholds[current_foothold_num][0]-2]) < ANGLE_TOL2:
									create_point = (self.points_reduced[self.footholds[current_foothold_num][0]-1][0] + 10 * math.cos(perpendicular_angle * math.pi/180), self.points_reduced[self.footholds[current_foothold_num][0]-1][1] + 10 * math.sin(perpendicular_angle * math.pi/180))
									(cornerx, cornery, t, r, s ) = self.intersectLines(self.points_reduced[self.footholds[current_foothold_num][0]-1], create_point, self.points_reduced[self.footholds[current_foothold_num][0]+1], self.points_reduced[self.footholds[current_foothold_num][0]])
									self.hidden_corners.append((cornerx, cornery))
									#hidden corners are predicted based on the assumption that the corner is a perpendicular one 
									if self.calc_distance((cornerx, cornery), self.points_reduced[self.footholds[current_foothold_num][0]+1]) > FOOTHOLD_MIN_LEN:
										step_coord = (cornerx + FOOTHOLD_MIN_LEN  * math.cos(math.pi/180 * self.seg_angle_list[self.footholds[current_foothold_num][0]]), cornery + FOOTHOLD_MIN_LEN * math.sin(math.pi/180 * self.seg_angle_list[self.footholds[current_foothold_num][0]])) 
										if self.calc_distance(step_coord, steps_coord[j][1]) > FOOTHOLD_MIN_LEN:
											steps_coord[j+1] = [self.footholds[current_foothold_num][0], step_coord]
											j += 1
						else:
							break
				else:
					break

			self.steps_coord = steps_coord
		if PAUSE:
			input("Press Enter to continue...")

	
	def find_point(self, seg_start_coord, offset, angle, range_index_low):
		#find the point in the data on a segment given that is an offset away from a starting point 
		find_x_estimate = seg_start_coord[0] + offset * math.cos(angle/180.0*math.pi)
		a = list(zip(*self.data_select[self.points_reduced_index[range_index_low] : self.points_reduced_index[range_index_low + 1]]))[0]
		b = a.index(min(a, key=lambda x:abs(x - find_x_estimate)))
		return self.data_select[b + self.points_reduced_index[range_index_low]]

	def rotate_angle(self, original_coord, angle):
		angle = angle*math.pi/180.
		rotation_matrix = numpy.array([(math.cos(angle), -math.sin(angle)), (math.sin(angle), math.cos(angle))])
		point_matrix = numpy.array(original_coord).transpose()
		result_matrix = numpy.dot(rotation_matrix, point_matrix).transpose()
		result = numpy.ndarray.tolist(result_matrix)
		result = [tuple(l) for l in result]
		return result

	def calc_distance(self, point1, point2):
		deltax = point1[0] - point2[0]
		deltay = point1[1] - point2[1]
		return math.sqrt(deltax**2 + deltay**2)

	def intersectLines(self, pt1, pt2, ptA, ptB ): 
		""" this returns the intersection of Line(pt1,pt2) and Line(ptA,ptB)
			
			returns a tuple: (xi, yi, valid, r, s), where
			(xi, yi) is the intersection
			r is the scalar multiple such that (xi,yi) = pt1 + r*(pt2-pt1)
			s is the scalar multiple such that (xi,yi) = pt1 + s*(ptB-ptA)
				valid == 0 if there are 0 or inf. intersections (invalid)
				valid == 1 if it has a unique intersection ON the segment    """

		DET_TOLERANCE = 0.00000001

		# the first line is pt1 + r*(pt2-pt1)
		# in component form:
		x1, y1 = pt1;   x2, y2 = pt2
		dx1 = x2 - x1;  dy1 = y2 - y1

		# the second line is ptA + s*(ptB-ptA)
		x, y = ptA;   xB, yB = ptB;
		dx = xB - x;  dy = yB - y;

		# we need to find the (typically unique) values of r and s
		# that will satisfy
		#
		# (x1, y1) + r(dx1, dy1) = (x, y) + s(dx, dy)
		#
		# which is the same as
		#
		#    [ dx1  -dx ][ r ] = [ x-x1 ]
		#    [ dy1  -dy ][ s ] = [ y-y1 ]
		#
		# whose solution is
		#
		#    [ r ] = _1_  [  -dy   dx ] [ x-x1 ]
		#    [ s ] = DET  [ -dy1  dx1 ] [ y-y1 ]
		#
		# where DET = (-dx1 * dy + dy1 * dx)
		#
		# if DET is too small, they're parallel
		#
		DET = (-dx1 * dy + dy1 * dx)

		if math.fabs(DET) < DET_TOLERANCE: return (0,0,0,0,0)

		# now, the determinant should be OK
		DETinv = 1.0/DET

		# find the scalar amount along the "self" segment
		r = DETinv * (-dy  * (x-x1) +  dx * (y-y1))

		# find the scalar amount along the input line
		s = DETinv * (-dy1 * (x-x1) + dx1 * (y-y1))

		# return the average of the two descriptions
		xi = (x1 + r*dx1 + x + s*dx)/2.0
		yi = (y1 + r*dy1 + y + s*dy)/2.0
		return ( xi, yi, 1, r, s )

	def points_360degrees(self):
		#function that shares lidar data between Lidar thread and window thread 
		data_buffer_lock.acquire()
		try:
			if len(data_list_buffer) > 15:
				# print(len(data_list_buffer))
				self.polar_points.clear()
				self.polar_points_new.clear()
				for i in data_list_buffer[0:16]:
					self.polar_points[len(self.polar_points):len(i)] = i
				data_list_buffer[0:16] = []
				return True
			else:
				return False
		finally:
			data_buffer_lock.release()

	def draw_points(self):
		self.X,self.Y = self.SCREEN_SIZE[0]/10 , self.SCREEN_SIZE[0]/10
		self.k = self.SCREEN_SIZE[1]/2000
		line_list = []
		foothold_index_list = [x[0] for x in self.footholds]
		self.screen.fill((84,106,124))
		for p in self.data_select:
			x,y = p
			x,y = x*self.k+self.X,-y*self.k+self.Y
			pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 4)
		for index in range(len(self.points_reduced)):
			(xr, yr) = self.points_reduced[index]
			xr, yr = xr*self.k+self.X, -yr*self.k+self.Y 
			pygame.draw.circle(self.screen, 0x00ff00, (int(xr), int(yr)), 4)
			if index != len(self.points_reduced) - 1:
				(x1, y1) = self.points_reduced[index]
				x1, y1 = x1*self.k+self.X, -y1*self.k+self.Y 
				(x2, y2) = self.points_reduced[index+1]
				x2, y2 = x2*self.k+self.X, -y2*self.k+self.Y 
				if index in foothold_index_list:
					pygame.draw.line(self.screen, (0, 255, 0), [x1, y1], [x2, y2], 2)
				else:
					pygame.draw.line(self.screen, (255, 0, 0), [x1, y1], [x2, y2], 2)
		if self.redundant_cluster:
			for point_index in self.redundant_cluster:
				(xd, yd) = point_index
				xd, yd = xd*self.k+self.X, -yd*self.k+self.Y
				pygame.draw.circle(self.screen, 0xCD5437, (int(xd), int(yd)), 4)
		if self.steps_coord:
			for g in self.steps_coord.values():
				(xs, ys) = g[1]
				xs, ys = xs*self.k+self.X, -ys*self.k+self.Y 
				pygame.draw.circle(self.screen, 0xFFA233, (int(xs), int(ys)), 8)
			if len(self.steps_coord.values()) > 1:
				a = round(self.steps_coord[1][1][0] - self.steps_coord[0][1][0], 2)
				b = round(self.steps_coord[1][1][1] - self.steps_coord[0][1][1], 2)
				S2 = str(a) + "cm                                         " + str(b) + "cm"
				text = self.font.render(S2, True, (255,255,255))
				self.screen.blit(text, (600,1200))
		S1 = "横坐标变化：                                 纵坐标变化:"
		text = self.font.render(S1, True, (255,255,255))
		self.screen.blit(text, (200,1200))
		if self.hidden_corners:
			for (xh, yh) in self.hidden_corners:
				xh, yh = xh*self.k+self.X, -yh*self.k+self.Y 
				pygame.draw.circle(self.screen, 0xAAF904, (int(xh), int(yh)), 4)
		pygame.draw.circle(self.screen, 0xff00ff, (int(self.X), int(self.Y)), 50)
		pygame.draw.line(self.screen, (0, 0, 0), [self.X, self.Y], [self.X - 50*math.sin(self.lidar_angle), self.Y - 50*math.cos(self.lidar_angle)], 2)


	def warning(self, warning_str):
		self.X,self.Y = self.SCREEN_SIZE[0]/2 , self.SCREEN_SIZE[1]/2
		self.screen.fill((84,106,124))
		warning_text = self.font.render(warning_str, True, (255,255,255))
		width = warning_text.get_width()
		height = warning_text.get_height()
		self.screen.blit(self.warning_image, (self.X-self.warning_image.get_width()/2, self.Y-self.warning_image.get_height()/2 - self.Shift_W))
		self.screen.blit(warning_text, (self.X-width/2, self.Y-height/2 + self.Shift_T))

	def Screen_size_change(self, s):
		self.SCREEN_SIZE = s
		self.X,self.Y = self.SCREEN_SIZE[0]/2 , self.SCREEN_SIZE[1]/2
		self.k = self.SCREEN_SIZE[1]/12000
		self.point = pygame.image.load('point_white_2.png').convert_alpha()
		self.W,self.H = self.point.get_width()/2 , self.point.get_height()/2
		self.warning_image = pygame.image.load('warning_2.png').convert_alpha()
		self.Shift_T = 25
		self.Shift_W = 100
		self.font = pygame.font.Font("PingFang.ttf", int(35*self.SCREEN_SIZE[0]/1400))
		self.screen = pygame.display.set_mode(self.SCREEN_SIZE, RESIZABLE, 32)
		self.D = 1

	def XYK_change(self):
		self.X,self.Y = self.SCREEN_SIZE[0]/2 , self.SCREEN_SIZE[1]/2
		self.k = self.SCREEN_SIZE[1]/12000

	def XY_move(self, move_tion):
		self.X, self.Y = self.X + self.D * move_tion[0] , self.Y + self.D * move_tion[1]

	def Y_change(self, C):
		self.Y += C

	def X_change(self, C):
		self.X += C

	def k_change(self, C):
		if C == 0:
			self.k = self.SCREEN_SIZE[1]/12000
		else:
			a = 10
			self.k = self.k * (a + C)/a

	def run(self):
		while True:
			# print("projection")
			if self.points_360degrees():
				self.projection()
			#self.projection()

			state_lock.acquire()
			try:
				state_buffer = state_mode
			finally:
				state_lock.release()

			if not state_buffer:
				if self.drawBoln:
					self.draw_points()
			else:
				self.warning(state_buffer[0])
			if self.drawBoln:
				pygame.display.flip()

class IMU_data_get(serial.Serial, threading.Thread):
	global lidar_angle_buffer

	def __init__(self, threadID, name, S, B = 115200):
		threading.Thread.__init__(self)
		thread_queue[threadID] = self
		# serial.Serial.__init__(self, serial_port, Baud)
		self.K = 180 / math.pi
		self.serial_port = S
		self.baud = B
		self.threadID = threadID
		self.name = name
		self.start_time = 0
		self.lidar_angle = 0
		# init the serial

		try:
			serial.Serial.__init__(self, self.serial_port, self.baud, timeout = 0.5)
		except serial.serialutil.SerialException:
			state_lock_IMU.acquire()
			try:
				state_mode_IMU.clear()
				state_mode_IMU.append("串口设备  "+self.serial_port+"  连接失败")
			finally:
				state_lock_IMU.release()
				raise serial.serialutil.SerialException

	def restart(self):
		try:
			serial.Serial.__init__(self, self.serial_port, self.baud)
		except serial.serialutil.SerialException:
			state_lock_IMU.acquire()
			try:
				state_mode_IMU.clear()
				state_mode_IMU.append("串口设备  "+self.serial_port+"  连接失败")
			finally:
				state_lock_IMU.release()
				raise serial.serialutil.SerialException

	def Head_check(self):
		response = self.read()
		if response == b'\xfa':
			response = self.read()
			if response == b'\xff':
				MID = self.read()
				if MID == b'\x36':
					is_data = True
				else:
					is_data = False
			else:
				is_data = False
		else:
			is_data = False
		return is_data

	def get_data(self):
		# IO intensive
		'''
		read data from imu. The function assumes the IMU data form with start code: 1020, 2038, and 8020. Only Euler angle data is used for calculating lidar angle. 
		Other 2 data packets are not used. 
		'''
		data_length = self.read()
		data_length = binascii.hexlify(data_length)
		data_length = data_length.decode(errors = 'ignore')
		data_length = int(data_length, 16)
		data = self.read(data_length)
		data = binascii.hexlify(data)
		count_data = 0
		while count_data < data_length:
			data_ID = self.read(2)
			print_ID = binascii.hexlify(data_ID)
			count_data += 2
			if data_ID == b'\x10\x20':
				packet_count_length = self.read()
				count_data += 1
				packet_count_length = binascii.hexlify(packet_count_length)
				packet_count_length = packet_count_length.decode(errors = 'ignore')
				packet_count_length = int(packet_count_length, 16)
				packet_count = self.read(packet_count_length)
				count_data += packet_count_length
			if data_ID == b'\x20\x38':
				data_size = self.read()
				count_data += 1
				data_size = binascii.hexlify(data_size)
				data_size = data_size.decode(errors = 'ignore')
				data_size = int(data_size, 16)
				euler = []
				for i in range(math.floor(data_size/4)):
					item = self.read(4)
					count_data += 4
					item = str(binascii.hexlify(item))[2:-1]
					item = struct.unpack('!f', bytes.fromhex(item))[0]
					euler.append(item)
				self.lidar_angle = (time.time(), euler[1] + LIDAR_OFFSET)
			if data_ID == b'\x80\x20':
				data_size = self.read()
				count_data += 1
				data_size = binascii.hexlify(data_size)
				data_size = data_size.decode(errors = 'ignore')
				data_size = int(data_size, 16)
				rate_of_turn = []
				for i in range(math.floor(data_size/4)):
					item = self.read(4)
					count_data += 4
					item = str(binascii.hexlify(item))[2:-1]
					item = struct.unpack('!f', bytes.fromhex(item))[0]
					rate_of_turn.append(item)
			if data_ID == b'\x40\x20':
				data_size = self.read()
				count_data += 1
				data_size = binascii.hexlify(data_size)
				data_size = data_size.decode(errors = 'ignore')
				data_size = int(data_size, 16)
				acceleration = []
				for i in range(math.floor(data_size/4)):
					item = self.read(4)
					count_data += 4
					item = str(binascii.hexlify(item))[2:-1]
					item = struct.unpack('!f', bytes.fromhex(item))[0]
					acceleration.append(item)
		check_sum = self.read()
		check_sum = binascii.hexlify(check_sum)

	def run(self):
		# IMU_read_lock_2.acquire()
		while True:
			try:
				if (not state_mode_IMU):
					# print("angle")
					# IMU_read_lock_1.acquire()
					# try:
					#     self.Gravity_angle()
					#     IMU_read_lock_2.release()
					# finally:
					#     IMU_read_lock_1.release()
					#     IMU_read_lock_2.acquire()
					# while Flag.get():
					#     pass
					is_data = self.Head_check()
					if is_data:
						self.get_data()
						IMU_buffer_lock.acquire()
						try:
							lidar_angle_buffer.append(self.lidar_angle)
							if len(lidar_angle_buffer) > LIDAR_BUFFER_LEN:
								lidar_angle_buffer.pop(0)
						finally:
							IMU_buffer_lock.release()
			except serial.serialutil.SerialException:
				state_lock.acquire()
				try:
					state_mode.clear()
					state_mode.append("串口设备  "+self.serial_port+"  丢失  按下command+R重连")
				finally:
					state_lock.release()


def Event_handle(obj):
	while True:
		event = pygame.event.wait()

		if event.type == QUIT:
			exit()

		if event.type == VIDEORESIZE:
			obj.Screen_size_change(event.size)

		if event.type == KEYDOWN:
			if event.key == K_q:
				if pygame.key.get_mods() & pygame.KMOD_META:
					exit()
			elif event.key == K_r:
				if pygame.key.get_mods() & pygame.KMOD_META:
					th = Thread_restart()
					th.setDaemon(True)
					th.start()
			elif event.key == K_c:
				if pygame.key.get_mods() & pygame.KMOD_META:
					obj.XYK_change()
			elif event.key == K_UP:
				obj.Y_change(-10)
			elif event.key == K_DOWN:
				obj.Y_change(10)
			elif event.key == K_LEFT:
				obj.X_change(-10)
			elif event.key == K_RIGHT:
				obj.X_change(10)
			elif event.key == K_PLUS:
				if pygame.key.get_mods() & pygame.KMOD_META:
					obj.k_change(1)
			elif event.key == K_MINUS:
				if pygame.key.get_mods() & pygame.KMOD_META:
					obj.k_change(-1)
			elif event.key == K_0:
				if pygame.key.get_mods() & pygame.KMOD_META:
					obj.k_change(0)

		if event.type == MOUSEBUTTONDOWN:
			if event.button == 5:
				obj.k_change(-1)
			elif event.button == 4:
				obj.k_change(1)

		if event.type == MOUSEMOTION:
			if event.buttons[0]:
				obj.XY_move(event.rel)

class Thread_start(threading.Thread):

	def run(self):

		Not_started = True
		while Not_started:
			try:
				radar_serial_thread_1 = radar_serial_thread(1, 'Thread-1', '/dev/tty.SLAB_USBtoUART')
				radar_serial_thread_1.setDaemon(True)
				radar_serial_thread_1.start()
				state_lock.acquire()
				try:
					state_mode.clear()
				finally:
					state_lock.release()
				Not_started = False
			except serial.serialutil.SerialException:
				pass
		print("start thread end")

class Thread_IMU_start(threading.Thread):

	def run(self):

		Not_started = True
		while Not_started:
			try:
				IMU = IMU_data_get(3, 'Thread-3', '/dev/tty.SLAB_USBtoUART140')
				IMU.setDaemon(True) 
				IMU.start()
				state_lock_IMU.acquire()
				try:
					state_mode_IMU.clear()
				finally:
					state_lock_IMU.release()
				Not_started = False
			except serial.serialutil.SerialException:
				pass
		print("start Thread-3 end")

class Thread_restart(threading.Thread):

	def run(self):

		Not_started = True
		while Not_started:
			try:
				thread_queue[1].restart()
				state_lock.acquire()
				try:
					state_mode.clear()
				finally:
					state_lock.release()
				Not_started = False
			except serial.serialutil.SerialException:
				pass
		print("restart done")

window_thread_1 = window_thread(2, 'Thread-2', DRAWBOLN)
window_thread_1.setDaemon(True)

start2thread = Thread_start()
start2thread.setDaemon(True)

start_IMU_thread = Thread_IMU_start()
start_IMU_thread.setDaemon(True)

window_thread_1.start()
time.sleep(0.9)
start2thread.start()
time.sleep(0.9)
start_IMU_thread.start()

if __name__ == '__main__':
	if DRAWBOLN:
		Event_handle(window_thread_1)
	else:
		pass

# -*- coding: utf-8 -*-

import serial, threading, math, time, pygame, copy
from sys import exit
from pygame.locals import *

state_mode = []
result_ = []
thread_queue = {}

a = []
b = []

data_buffer_lock = threading.Lock()
state_lock = threading.Lock()

class serial_read_error(Exception):
    pass
class serial_read_time_out(Exception):
    pass

# class

class radar_serial_thread(serial.Serial, threading.Thread):
    global state_mode, a ,b

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
            while self.read() != b'\xFF':
                pass
                # if (time.time() - self.start_time) > 5:
                #     raise serial_read_time_out

            # length = self.twobyte2int()-3
            # print("1")

            #address and type code check
            if self.read() == b'\xFF':
                return True
            else:
                raise serial_read_error

        except serial_read_error:
            print("Serial data don't accord with the protocol ...... Data ignored")
        except serial_read_time_out:
            print("Head_check time out, Please comfirm the device is legal")
            raise serial_read_time_out

    def fourbyte2int(self):
        # IO intensive
        buffer = self.read(4)
        int_number = buffer[0]*16777216 + buffer[0]*65536 + buffer[0]*256 + buffer[0]
        return int_number

    def data_decode(self, L):# 解码运算一帧数据
        # IO intensive
        state_lock.acquire()
        try:
            if len(state_mode)>0:
                state_mode.clear()
        finally:
            state_lock.release()
        data_buffer_lock.acquire()
        try:
            a.append(self.fourbyte2int())
            b.append(self.fourbyte2int())
        finally:
            data_buffer_lock.release()

    def run(self):
        while True:
            try:
                if (not state_mode) or state_mode[0] != "串口设备  "+self.serial_port+"  不存在":
                    if self.Head_check():
                        self.data_decode()
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
    global state_mode, a, b

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        thread_queue[threadID] = self
        self.threadID = threadID
        self.name = name
        self.points = []
        self.K = math.pi/180
        self.points_analyse = []
        self.data_xy_list = []
        self.N = 500
        # self.differential_xy = []


        self.SCREEN_SIZE = (2880, 1700)
        # pygame window init
        pygame.init()
        self.screen = pygame.display.set_mode(self.SCREEN_SIZE, RESIZABLE, 32)
        pygame.display.set_caption("Radar Points")

        # 载入图片
        self.point = pygame.image.load('source/point.png').convert_alpha()
        self.warning_image = pygame.image.load('source/warning.png').convert_alpha()

        # 字体设置
        self.font = pygame.font.Font("source/PingFang.ttf", 70)

        # point图片长款补偿
        self.W,self.H = self.point.get_width()/2 , self.point.get_height()/2
        self.X,self.Y = self.SCREEN_SIZE[0]*3/4 , self.SCREEN_SIZE[1]/2
        self.k = self.SCREEN_SIZE[1]/180000
        self.Shift_T = 50    #文本Y轴偏移
        self.Shift_W = 200   #标志Y轴偏移

        self.D = 2

    def draw_points(self):
        global a, b
        self.screen.fill((84,106,124))
        t = self.X
        a = a[-self.N:]
        for p in range(len(a)):
            x = -a[p]
            t -= 5
            self.screen.blit(self.point, (t - self.W, x - self.H))
        t = self.X
        b = b[-self.N:]
        for p in range(len(b)):
            x = -b[p]
            t -= 5
            self.screen.blit(self.point, (t - self.W, x - self.H))

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
        self.point = pygame.image.load('source/point_white_2.png').convert_alpha()
        self.W,self.H = self.point.get_width()/2 , self.point.get_height()/2
        self.warning_image = pygame.image.load('source/warning_2.png').convert_alpha()
        self.Shift_T = 25
        self.Shift_W = 100
        self.font = pygame.font.Font("source/PingFang.ttf", int(35*self.SCREEN_SIZE[0]/1400))
        self.screen = pygame.display.set_mode(self.SCREEN_SIZE, RESIZABLE, 32)
        self.D = 1

    def XYK_change(self):
        self.X,self.Y = self.SCREEN_SIZE[0]*3/4 , self.SCREEN_SIZE[1]/2
        self.k = self.SCREEN_SIZE[1]/180000

    def XY_move(self, move_tion):
        self.X, self.Y = self.X+ self.D*move_tion[0] , self.Y+ self.D*move_tion[1]

    def k_change(self, C):
        if C == 0:
            self.k = self.SCREEN_SIZE[1]/12000
        else:
            a = 10
            self.k = self.k * (a + C)/a

    def run(self):
        while True:
            # print("projection")

            state_lock.acquire()
            try:
                state_buffer = state_mode
            finally:
                state_lock.release()

            if not state_buffer:
                data_buffer_lock.acquire()
                try:
                    self.draw_points()
                finally:
                    data_buffer_lock.release()
            else:
                self.warning(state_buffer[0])

            pygame.display.flip()

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
                obj.XY_move((0,-5))
            elif event.key == K_DOWN:
                obj.XY_move((0,5))
            elif event.key == K_LEFT:
                obj.XY_move((-5,0))
            elif event.key == K_RIGHT:
                obj.XY_move((5,0))
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



window_thread_1 = window_thread(2, 'Thread-2')
window_thread_1.setDaemon(True)
window_thread_1.start()


start2thread = Thread_start()
start2thread.setDaemon(True)
start2thread.start()

# window_thread_1 = window_thread(2, 'Thread-2')
# radar_serial_thread_1 = radar_serial_thread(1, 'Thread-1', '/dev/tty.SLAB_USBtoUART')
#
# window_thread_1.setDaemon(True)
# radar_serial_thread_1.setDaemon(True)
#
# window_thread_1.start()
# radar_serial_thread_1.start()

if __name__ == '__main__':
    Event_handle(window_thread_1)

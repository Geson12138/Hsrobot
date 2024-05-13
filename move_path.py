# coding=utf-8
import os
import queue
import random
import re
import sys
import threading
import time
import csv
import traceback
from datetime import datetime

from src.CPS import CPSClient

#################刀具，用户坐标#################################################
cps = CPSClient('192.168.8.116')
TCPName = 'TCP_1'
UCS = [0, 0, 0, 0, 0, 0]


####################################################################################
def waitMoveDone1():
    while True:
        nRet = cps.HRIF_ReadTrackProcess()
        print(nRet)
        if abs(float(nRet[1]) - 1) != 0:
            time.sleep(0.01)
            continue
        if abs(float(nRet[1]) - 1) == 0:
            time.sleep(0.6)
            break
        time.sleep(0.1)

def print_time():
    time_string = datetime.now()
    print(time_string)
def waitForceDone():
    while True:
        nRet = cps.HRIF_ReadForceControlState()
        print("waitForceDone HRIF_ReadForceControlState->%s" % nRet)
        if float(nRet[1]) == 2:
            time.sleep(0.2)
            break
        time.sleep(0.2)


def wait_Force_OFF_Done():
    while True:
        nRet = cps.HRIF_ReadForceControlState()
        print(nRet)
        if float(nRet[1]) == 0:
            time.sleep(0.2)
            break
        time.sleep(0.2)

def setup_force():
    # 设置力控工具坐标
    print(cps.HRIF_SetForceToolCoordinateMotion(1))
    mass = [10, 10, 10, 4, 4, 4]
    # 设置mass
    res_demo = cps.HRIF_SetMassParams(mass)
    print("cps.HRIF_SetMassParams = %s" % res_demo)

    # 设置探寻距离200mm,上下共200mm
    print(cps.HRIF_SetForceDistanceLimit(100, 2))
    # 设置切向力(X,Y方向力)大于20N时,机器人向Z方向抬起手臂,直到切向力小于Min(10N)后,力控恢复正常探寻
    print(cps.HRIF_SetTangentForceBounds(25, 10, 50))
    # 设置避障模式
    print(cps.HRIF_SetForceControlStrategy(2))
    # 探寻自由度：Z轴
    print(cps.HRIF_SetControlFreedom([0, 0, 1, 0, 0, 0]))
    # 探寻力Z方向：10
    print(cps.HRIF_SetForceControlGoal([0, 0, 10, 0, 0, 0]))
    # 设置力控探寻的最大速度
    print(cps.HRIF_SetMaxSearchVelocities(20, 10))
    # 力控清零
    print(cps.HRIF_SetAppForceZero())
    time.sleep(1)
    set_force_zero_res = cps.HRIF_SetForceZero()
    print("cps.HRIF_SetForceZero() %s" % set_force_zero_res)
    if set_force_zero_res[0] != '0':
        sys.exit()

    # 力传感器开启
    # print(cps.HRIF_SetForceControlState(1))
    waitForceDone()
    print("探寻力设置完毕")


def getPointFromFile(trackfile):
    csvFile = open(trackfile, "r")
    reader = csv.reader(csvFile)
    start = time.perf_counter()

    pointsCNT = 0
    points = []
    positions = []
    ucsPoint = []

    # move to first point
    for i, row in enumerate(reader):
        # if i % 3 != 0:
        #     continue
        rawUcsTcp = str(row[0]).split()  # 按空格分
        # print(rawUcsTcp)
        ucsPoint = []
        ucsPoint = cps.HRIF_UcsTcp2Base(rawUcsTcp, [0, 0, 0, 0, 0, 0], UCS)
        # 删除首位的错误码
        del ucsPoint[0]
        if i == 0:
            initPosition = ucsPoint
        # print(ucsPoint)
        positions += ucsPoint
        points.append(ucsPoint)
        pointsCNT += 1
    return positions

def check_robot_is_standBy():
    count = 0
    while True:
        # ReadRobotState,OK,0,1,0,0,0,0,0,0,0,1,1,1,1,;
        robot_state = cps.HRIF_ReadRobotState()
        print(robot_state)
        res_str = ""
        if robot_state[1] == '1':
            res_str = "is running"
        elif robot_state[2] == '0':
            res_str = "unable"
        elif robot_state[3] != '0':
            res_str = robot_state[3] + " code:" + robot_state[4] + " index:" + robot_state[5]
        elif robot_state[6] != '0':
            res_str = "bao zha!"
        elif robot_state[7] == '1':
            res_str = "pause!"
        elif robot_state[8] == '1':
            res_str = "ji stop!"
        elif robot_state[9] == '1':
            res_str = "safe light control!"
        elif robot_state[10] == '0':
            res_str = "no electrify!"
        elif robot_state[11] == '0':
            res_str = "no electrify box!"
        elif robot_state[12] == '0':
            res_str = "way point is no finish!"
        else:
            res_str = "ok"
        print("res_str->%s" % res_str)
        if res_str != "ok":
            count += 1
            time.sleep(0.1)
            if count >= 30:
                return -1
            else:
                continue
        else:
            return res_str


def MovePath(pathfile, name_str):
    print('test traject:' + pathfile)
    trackfile = pathfile
    csvFile = open(trackfile, "r")
    # reader=csv.reader(csvFile)
    reader = csvFile
    start = time.perf_counter()

    pointsCNT = 0
    points = []
    positions = []
    ucsPoint = []

    tcp = cps.HRIF_ReadTCPByName(TCPName)
    del tcp[0]
    # move to first point
    print('read file')
    # for i,row in enumerate(reader):
    #      if i%3!=0:
    #         continuefor i in reader.readline:

    for i in reader:
        print(i)
        row = i
        # print(row)
        # rawUcsTcp = row[:len(row)-1]#str(row).split()#按空格分
        rawUcsTcp = str(row).split(" ")  # 按空格分
        # rawUcsTcp=rawUcsTcp[0:len(rawUcsTcp)-1]
        print(rawUcsTcp)
        ucsPoint = []
        ucsPoint = cps.HRIF_UcsTcp2Base(rawUcsTcp, [0, 0, 0, 0, 0, 0], UCS)
        del ucsPoint[0]
        print(ucsPoint)
        if i == 0:
            initPosition = ucsPoint
        positions += ucsPoint
        points.append(ucsPoint)
        pointsCNT = pointsCNT + 1

        start_point = points[0][0:6]
        print("start_point: %s" % start_point)
              # 第一条轨迹计算完成后，开启力传感器
        nRet = cps.HRIF_ReadForceControlState()
        if float(nRet[1]) == 0:
            # 笛卡尔运动到第一条轨迹第一个点
            print("start_point: %s, cps.WayPointEx->%s" % (
            start_point, cps.WayPointEx(1, start_point, [0, 0, 0, 0, 0, 0], tcp,
                                        [0, 0, 0, 0, 0, 0], 200, 2500, 50, 1, 0, 0, 0, ' ')))
            time.sleep(0.02)
            cps.waitMoveDone()
            print_time()


    count = 0
    tmp_point = []
    for item in positions:
        count = count + 1
        tmp_point.append(item)
        if count % 6 == 0:
            print(cps.HRIF_PushMovePathL(name_str, tmp_point))
            tmp_point = []
        if count % 6000 == 0:
            print("MovePathL calculation! 1000 points")
            print(cps.HRIF_EndPushMovePath(name_str))

    print("A file of MovePath stop calculation!")
    print(cps.HRIF_EndPushMovePath(name_str))

    # print (cps.HRIF_PushMovePaths('st1',1,pointsCNT,positions))

    end = time.perf_counter()
    print('time(s): ' + str(end - start))
    print("Total points of a file: " + str(pointsCNT))
    time.sleep(1)

    print("cps.HRIF_MovePathL->%s = %s" % (name_str, cps.HRIF_MovePathL(name_str)))
    time.sleep(0.2)
    waitMoveDone1()
    time.sleep(3)
    print_time()
    print("cps.HRIF_MovePathL->%s finished" % name_str)
    cps.HRIF_SetForceControlState(0)
    wait_Force_OFF_Done()
    # cps.waitMoveDone()


# 查找给定文件夹下面所有
def find_file_by_pattern(base=".", pattern='.*', ignore_pattern='', circle=True):
    re_file = re.compile(pattern)
    # re_ignore = re.compile(ignore_pattern)
    # print(re_ignore)
    if base == ".":
        base = os.getcwd()

    final_file_list = []
    # print(base)
    cur_list = os.listdir(base)

    for item in cur_list:
        if item == ".2d":
            continue

        full_path = os.path.join(base, item)
        if full_path.endswith(".2d") or \
                full_path.endswith(".bmp") or \
                full_path.endswith(".wpt") or \
                full_path.endswith(".dot"):
            continue

        # print full_path
        nextpath = base + '\\' + str(item)

        bfile = os.path.isfile(nextpath)
        # print(str(bfile)+':'+nextpath)

        if bfile:
            final_file_list.append(nextpath)
        else:
            final_file_list += find_file_by_pattern(nextpath, pattern, ignore_pattern)

    return final_file_list


all_points_load_flag = False
bForceControl = False
bFirstPathMoving = False
path_name_q = queue.Queue(10)
start_point_q = queue.Queue(10)
point_list = []
name_list = []
tmp_point = []
# 定义锁
lock = threading.Lock()


# 生产者
class Productor(threading.Thread):
    # 覆写run方法
    def __init__(self, i):
        super().__init__()
        self.i = i

    def run(self):
        global all_points_load_flag
        global bForceControl
        global bFirstPathMoving
        for points_index in range(len(point_list)):
            print("########################################## push and compute start ######################################################")
            print_time()
            lock.acquire()
            name_str = name_list[points_index]
            lock.release()
            print("cps.HRIF_InitMovePathL->%s = %s" % (name_str, cps.HRIF_InitMovePathL(name_str, 20, 500, 500, 'Base', 'TCP_1')))
            points = point_list[points_index]
            start_point = points[0:6]
            print_time()
            print("HRIF_PushMovePaths->%s" % cps.HRIF_PushMovePaths(name_str, 1, len(points)/6, points))
            print_time()
            print("HRIF_EndPushMovePath->%s = %s" % (name_str, cps.HRIF_EndPushMovePath(name_str)))
            lock.acquire()
            path_name_q.put(name_str)
            start_point_q.put(start_point)
            lock.release()
            print_time()

            # 第一条轨迹计算完成后，开启力传感器
            nRet = cps.HRIF_ReadForceControlState()
            if float(nRet[1]) == 0:
                # 笛卡尔运动到第一条轨迹第一个点
                print("start_point: %s, cps.WayPointEx->%s" % (start_point, cps.WayPointEx(1, start_point, [0, 0, 0, 0, 0, 0], tcp,
                       [0, 0, 0, 0, 0, 0], 200, 2500, 50, 1, 0, 0, 0, ' ')))
                time.sleep(0.02)
                cps.waitMoveDone()
                print_time()

                cps.HRIF_SetForceControlState(1)
                waitForceDone()
                time.sleep(0.2)
                bForceControl = True
                time.sleep(2)

            print("########################################## push and compute end ######################################################")
        all_points_load_flag = True


# 消费者
class Consumer(threading.Thread):
    def __init__(self, i):
        super().__init__()
        self.i = i

    def run(self):
        global bForceControl
        while not all_points_load_flag or not path_name_q.empty():
            if not path_name_q.empty() and bForceControl:
            # if not path_name_q.empty():
                print("path_name_q.qsize()->%s" % path_name_q.qsize())
                lock.acquire()
                path_name = path_name_q.get()
                start_point = start_point_q.get()
                lock.release()
                print_time()

                # 笛卡尔运动到轨迹第一个点
                print("start_point: %s, cps.WayPointEx->%s" % (
                start_point, cps.WayPointEx(1, start_point, [0, 0, 0, 0, 0, 0], tcp,
                                    [0, 0, 0, 0, 0, 0], 20, 500, 50, 1, 0, 0, 0, ' ')))
                time.sleep(0.02)
                cps.waitBlendingDone()

                print("cps.HRIF_MovePathL->%s = %s" % (path_name, cps.HRIF_MovePathL(path_name)))
                time.sleep(0.2)
                waitMoveDone1()
                time.sleep(3)
                print_time()
                print("cps.HRIF_MovePathL->%s finished" % path_name)
        cps.HRIF_SetForceControlState(0)
        wait_Force_OFF_Done()
        bForceControl = False
        cps.waitMoveDone()
        cps.WayPointEx(1, [-500, 0, 500, 180, -20, 180], [156.728, -10.937, 103.336, -6.241, 42.891, 162.682], tcp,
                       [0, 0, 0, 0, 0, 0], 50, 500, 50, 0, 0, 0, 0, ' ')
        cps.waitMoveDone()


####################################################################################
if __name__ == '__main__':
    try:
        tcp = cps.HRIF_ReadTCPByName(TCPName)
        del tcp[0]
        print("tcp %s" % tcp)
        # file_list = find_file_by_pattern(r'E:\deyiwork\hans_SDK\力控plug\力控20220709\test2', '.*', '.2d')
        file_list = find_file_by_pattern(r'E:\deyiwork\hans_SDK\力控plug\力控20220709\path20230420', '.*', '.2d')
        print(file_list)
        print_time()

        cps.WayPointEx(1, [-500, 0, 300, 180, -20, 180], [156.728, -10.937, 103.336, -6.241, 42.891, 162.682], tcp,
                       [0, 0, 0, 0, 0, 0], 400, 500, 50, 0, 0, 0, 0, ' ')
        time.sleep(0.02)
        cps.waitBlendingDone()
        name_str = ''
        setup_force()

        for itemf in file_list:
            base_name = os.path.basename(itemf)
            name_str = base_name[0:len(base_name) - 4]
            print("name_str->%s" % name_str)
            points_from_file = getPointFromFile(itemf)
            point_list.append(points_from_file)
            name_list.append(name_str)

        points = point_list[0]
        start_point = points[0:6]
        print("start_point: %s" % start_point)

        # 第一条轨迹计算完成后，开启力传感器
        nRet = cps.HRIF_ReadForceControlState()
        print("HRIF_ReadForceControlState: %s" % nRet)

        if float(nRet[1]) != 0:
            nRet = cps.HRIF_ReadForceControlState()
            while float(nRet[1]) != 0:
                nRet = cps.HRIF_ReadForceControlState()
                time.sleep(1)
        if float(nRet[1]) == 0:
            # 笛卡尔运动到第一条轨迹第一个点
            print("start_point: %s, cps.WayPointEx->%s" % (
            start_point, cps.WayPointEx(1, start_point, [0, 0, 0, 0, 0, 0], tcp,
                                        [0, 0, 0, 0, 0, 0], 200, 2500, 50, 1, 0, 0, 0, ' ')))
            time.sleep(0.02)
            cps.waitMoveDone()

            time.sleep(10)

            print("cps.HRIF_InitMovePathL->%s = %s" % (
                name_str, cps.HRIF_InitMovePathL(name_str, 40, 200, 200, 'Base', 'TCP_1')))  #name_str, 60, 500, 500, 'Base', 'TCP_1'
            print_time()
            print("HRIF_PushMovePaths->%s" % cps.HRIF_PushMovePaths(name_str, 1, len(points) / 6, points))
            print_time()
            print("HRIF_EndPushMovePath->%s = %s" % (name_str, cps.HRIF_EndPushMovePath(name_str)))

            cps.HRIF_SetForceControlState(1)
            waitForceDone()
            time.sleep(2)

            print("cps.HRIF_MovePathL->%s = %s" % (name_str, cps.HRIF_MovePathL(name_str)))
            time.sleep(0.2)
            cps.waitMoveDone()
            time.sleep(3)
            print_time()
            print("cps.HRIF_MovePathL->%s finished" % name_str)
            cps.HRIF_SetForceControlState(0)
            wait_Force_OFF_Done()
        else:
            print("robot not standby!!!!!!!!!!!!!!!")





        # productor = Productor(1)
        # consumer = Consumer(1)
        # productor.start()
        # consumer.start()
    except Exception as e:
        traceback.print_exc()
        os._exit(0)
####################################################################################
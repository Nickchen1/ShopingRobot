__author__ ='Nick chen'

# coding:utf8
import time
import platform
import serial
import sys
from pathplanning.Pathplanning import PathPlanning
from maphandle.Map import  Map
from Shelfhandle.Shelf import Shelf
from warehouse.Warehouse import WareHouse
from robotarm.Robotarm import RobotArm




def serialinit():
    if 'Linux' in platform.platform():
        com1 = serial.Serial(
            '/dev/ttyACM0',
            115200,
            timeout=2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)

        com2 = serial.Serial(
            '/dev/ttyACM1',
            115200,
            timeout=2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)

    elif 'Windows' in platform.platform():
        com1 = serial.Serial(
            'COM60',
            115200,
            timeout=2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)

        com2 = serial.Serial(
            'COM55',
            115200,
            timeout=2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)

    else:
        raise RuntimeError('Unsupported platform.')

    time.sleep(4)

    com1_info = com1.readlines()
    com1_info = [line.decode().strip() for line in com1_info]

    if 'start' in com1_info:
        arm_com = com1
        wheel_com=com2

    elif 'wheel' in com1_info:
        wheel_com = com1
        arm_com = com2

    else:
        raise RuntimeError('Serial port 1 (ttyACM0) authentication failed')


    return wheel_com,arm_com

def isComplete(serial):
    while True:
        s = serial.readline()
        print "waiting for feedback...", s
        if "Complete" in s:
            if serial is arm_com:
                time.sleep(1)
                print "arm complete!"
                break
                return True
            elif serial is wheel_com:
                print "wheel complete!"
                break
                return True




if __name__=='__main__':
    map=Map()
    order_list = ['A','B','C','D']
    #order_list = map.getSearch_List("area")
    shelf=Shelf('Left')
    warehouse=WareHouse()
    robotarm = RobotArm()
    pathpalning=PathPlanning(map.getMap(),map.getStartpoint(),'N')
    wheel_com, arm_com = serialinit()
    arm_com.write(robotarm.armInit())
    time.sleep(15)
    
    print('Done Initialize Robot.')


    item_list = {"A": {"Left": "RedCube", "Middle": "YangLeDuo", "Right": "Badminton"},
                 "B": {"Left": "YellowCube", "Middle": "ShuangWaiWai", "Right": "SteelBall"},
                 "C": {"Left": "BlueCube", "Middle": "GreenCube", "Right": "Tennis"},
                 "D": {"Left": "Snow", "Middle": "Apple", "Right": "Sprite"}}


    #begin to catch
    special_position = []
    special_area = ""
    special_item = ""

    for area in order_list:
        item_pos = item_list[area]
        robotarm.initCatchOrder()
        for pos in map.getSearch_List("cargo"):
            item = item_pos[pos]
            if item is 'None':
                robotarm.delCatchOrder(pos)
                continue

            '''
            if item is "Apple":
                special_area = area
                special_position.append(pos)
                special_item = item
                robotarm.delCatchOrder(pos)
                continue
            '''

            # go to the catching point
            destination, Existpos = warehouse.getFirst_Existwarehouse(map, area, robotarm.getCatch_Order())
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), destination)
            catchpath = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(),"Catch")
            frontlist = pathpalning.afterTurnFront()
            wheel_com.write(str(pathlist) + str(catchpath)+ str(frontlist) + '1O')
            #print "go to the catching point"
            isComplete(wheel_com)

            # begin to catch
            send_list = robotarm.ArmMoveto(Existpos) + robotarm.ArmCatch(robotarm.getItemangle(item))
            arm_com.write(send_list)
            print "begin to catch"
            isComplete(arm_com)
            warehouse.catch_warehouse(area, Existpos)
            print "Nick is gonging to standard..."
            arm_com.write(robotarm.Standardpose())


            # out of catching point
            wheel_com.write(pathpalning.aferCatchToback())


            # go to the putting point
            point, level = shelf.getFirst_Emptyshelf(shelf.whereIsItem(item))
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), point)
            wheel_com.write(pathlist + '1O')
            print "go to the putting point"
            isComplete(wheel_com)


            # turn to correct direction
            putpath = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(),"Put")
            wheel_com.write(putpath + '1O')
            print "turn to correct direction"
            isComplete(wheel_com)


            # arm to get up/down
            arm_com.write(robotarm.ArmMoveto(level))
            time.sleep(3)

            # car go forward
            wheel_com.write(pathpalning.afterTurnFront())
   


            # finishing putting
            arm_com.write(robotarm.ArmEase())
            print "finishing putting arm"
            isComplete(arm_com)

            shelf.putShelf(map, pathpalning.getCurrentposition(), shelf.whereIsItem(item), level)
            print "get_shelf of ", area, ":", shelf.get_shelf(shelf.whereIsItem(item))
            wheel_com.write(pathpalning.aferCatchToback() + '1O')
            print "finishing putting wheel"
            isComplete(wheel_com)
            arm_com.write(robotarm.Standardpose())






    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), map.getStartpoint())
    wheel_com.write(pathlist + '1q')












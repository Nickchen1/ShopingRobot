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
from Classify_handle.pic import car_pic
import cv2
import os



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
    recognize_cargo_area = ['B','C','D','A']
    recognize_shelf_area = ['D','C','B']
    catch_area = ['A','B','C','D']
    map=Map()
    order_list = map.getSearch_List("area")
    shelf=Shelf('Left')
    warehouse=WareHouse()
    robotarm = RobotArm()
    classfiy_pic= car_pic()
    pathpalning=PathPlanning(map.getMap(),map.getStartpoint(),'N')
    wheel_com, arm_com = serialinit()
    arm_com.write(robotarm.armInit())
    
    print "loading model....."
    classfiy_pic.load_model()
    print('Done Initialize Robot.')
    
    a = raw_input("waiting for enter...")
    classfiy_pic.huojia_load_pipeline()
    print "opening camera....."

    time.sleep(11)



    
    item_list = {"A": {"Left": None, "Middle": None, "Right": None},
                "B": {"Left": None, "Middle": None, "Right": None},
                "C": {"Left": None, "Middle": None, "Right": None},
                "D": {"Left": None, "Middle": None, "Right": None}}
    

    wheel_com.write('2F')
    

    #recognize the shelf
    for c_area in recognize_shelf_area:
        shelf_pic_index = (x for x in range(1, 7))
        while (True):
            if shelf.getFirst_Uncheckshelf(c_area) == None:
                break
            #print "shelf.getFirst_Uncheckshelf:", shelf.getFirst_Uncheckshelf(c_area)
            facelist = pathpalning.TurnToFace(map, c_area, "Put")
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), shelf.getFirst_Uncheckshelf(c_area))
            wheel_com.write(facelist + pathlist + '1O')
            isComplete(wheel_com)
            time.sleep(0.5)
            classfiy_pic.huojia_take_pipeline(shelf_pic_index.next(), 0.5)
            shelf.CheckShelf(map, pathpalning.getCurrentposition(), c_area)
            time.sleep(0.5)

        shelf.set_shelf(c_area,classfiy_pic.huop(c_area,))
        print "shelf.get_shelf(by Nick chen)",shelf.get_shelf(c_area)

    classfiy_pic.huojia_close_pipeline()


    classfiy_pic.middle_load_cap()
    arm_com.write(robotarm.ArmMoveto('capture'))
    time.sleep(1)
    #recognize the cargo
    for index,c_area in enumerate(recognize_cargo_area):
        facelist = pathpalning.TurnToFace(map, c_area,"Capture")
        pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), map.getCaptureSite(c_area)[0])
        print pathlist ,facelist, c_area
        wheel_com.write(facelist + pathlist +  '1O')
        isComplete(wheel_com)

        time.sleep(1)
        classfiy_pic.middle_take_cap(index + 1)
        item_list[c_area] = classfiy_pic.recon(c_area)
        

    classfiy_pic.middle_close_cap()
    arm_com.write(robotarm.Standardpose())
    time.sleep(1)
        

    print "item_list(by Nick chen):",item_list



    #begin to catch
    special_position = []
    special_area = ""
    special_item = ""

    for area in catch_area:
        item_pos = item_list[area]
        robotarm.initCatchOrder()
        for pos in map.getSearch_List("cargo"):
            item = item_pos[pos]
            if item is 'None':
                robotarm.delCatchOrder(pos)
                continue

            if item is "Apple":
                special_area = area
                special_position.append(pos)
                special_item = item
                robotarm.delCatchOrder(pos)
                continue



            # go to the catching point
            destination, Existpos = warehouse.getFirst_Existwarehouse(map, area, robotarm.getCatch_Order())

            if destination is None:
                robotarm.delCatchOrder(pos)
                continue


            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), destination)
            catchpath = pathpalning.TurnToFace(map, area,"Catch")
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
            time.sleep(1)


            # out of catching point
            wheel_com.write(pathpalning.aferCatchToback())


            # go to the putting point
            point, level = shelf.getFirst_Emptyshelf(shelf.whereIsItem(item))
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), point)
            wheel_com.write(pathlist + '1O')
            print "go to the putting point"
            isComplete(wheel_com)


            # turn to correct direction
            putpath = pathpalning.TurnToFace(map, shelf.whereIsItem(item),"Put")
            wheel_com.write(putpath + '1O')
            print "turn to correct direction"
            isComplete(wheel_com)


            # arm to get up/down
            arm_com.write(robotarm.ArmMoveto(level))
            time.sleep(3)

            # car go forward
            wheel_com.write(pathpalning.afterTurnSlowFront())
            


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


            for new in item_list[shelf.whereIsItem(item)]:
                if new in shelf.whereIsItem(item):
                    destination, Existpos = warehouse.getFirst_Existwarehouse(map, shelf.whereIsItem(new), robotarm.getCatch_Order())

                    if destination is None:
                        robotarm.delCatchOrder(pos)
                        continue

                    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), destination)
                    catchpath = pathpalning.TurnToFace(map, shelf.whereIsItem(new), "Catch")
                    frontlist = pathpalning.afterTurnFront()
                    wheel_com.write(str(pathlist) + str(catchpath) + str(frontlist) + '1O')
                    # print "go to the catching point"
                    isComplete(wheel_com)

                    # begin to catch
                    send_list = robotarm.ArmMoveto(Existpos) + robotarm.ArmCatch(robotarm.getItemangle(new))
                    arm_com.write(send_list)
                    print "begin to catch"
                    isComplete(arm_com)
                    warehouse.catch_warehouse(shelf.whereIsItem(new), Existpos)
                    print "Nick is gonging to standard..."
                    arm_com.write(robotarm.Standardpose())
                    time.sleep(1)

                    # out of catching point
                    wheel_com.write(pathpalning.aferCatchToback())

                    # go to the putting point
                    point, level = shelf.getFirst_Emptyshelf(shelf.whereIsItem(new))
                    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), point)
                    wheel_com.write(pathlist + '1O')
                    print "go to the putting point"
                    isComplete(wheel_com)

                    # turn to correct direction
                    putpath = pathpalning.TurnToFace(map, shelf.whereIsItem(new), "Put")
                    wheel_com.write(putpath + '1O')
                    print "turn to correct direction"
                    isComplete(wheel_com)

                    # arm to get up/down
                    arm_com.write(robotarm.ArmMoveto(level))
                    time.sleep(3)

                    # car go forward
                    wheel_com.write(pathpalning.afterTurnSlowFront())

                    # finishing putting
                    arm_com.write(robotarm.ArmEase())
                    print "finishing putting arm"
                    isComplete(arm_com)

                    shelf.putShelf(map, pathpalning.getCurrentposition(), shelf.whereIsItem(new), level)
                    print "get_shelf of ", area, ":", shelf.get_shelf(shelf.whereIsItem(new))
                    wheel_com.write(pathpalning.aferCatchToback() + '1O')
                    print "finishing putting wheel"
                    isComplete(wheel_com)
                    arm_com.write(robotarm.Standardpose())






    if special_item is 'Apple':

        # go to the catching point
        destination, Existpos = warehouse.getFirst_Existwarehouse(map, special_area, special_position)
        pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), destination)
        catchpath = pathpalning.TurnToFace(map, special_area,"Catch")
        frontlist = pathpalning.afterTurnFront()
        wheel_com.write(str(pathlist) + str(catchpath)+ str(frontlist) + '1O')
        #print "go to the catching point"
        isComplete(wheel_com)

        # begin to catch
        send_list = robotarm.ArmMoveto(Existpos) + robotarm.ArmCatch(robotarm.getItemangle(special_item))
        arm_com.write(send_list)
        print "begin to catch"
        isComplete(arm_com)
        warehouse.catch_warehouse(special_area, Existpos)
        print "Nick is gonging to standard..."
        arm_com.write(robotarm.Standardpose())


        # out of catching point
        wheel_com.write(pathpalning.aferCatchToback())


        # go to the putting point
        point, level = shelf.getFirst_Emptyshelf(shelf.whereIsItem(special_item))
        pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), point)
        wheel_com.write(pathlist + '1O')
        print "go to the putting point"
        isComplete(wheel_com)


        # turn to correct direction
        putpath = pathpalning.TurnToFace(map, shelf.whereIsItem(special_item),"Put")
        wheel_com.write(putpath + '1O')
        print "turn to correct direction"
        isComplete(wheel_com)


        # arm to get up/down
        arm_com.write(robotarm.ArmMoveto(level))
        time.sleep(3)

        # car go forward
        wheel_com.write(pathpalning.afterTurnSlowFront())
        


        # finishing putting
        arm_com.write(robotarm.ArmEase())
        print "finishing putting arm"
        isComplete(arm_com)

        shelf.putShelf(map, pathpalning.getCurrentposition(), shelf.whereIsItem(special_item), level)
        print "get_shelf of ", area, ":", shelf.get_shelf(shelf.whereIsItem(special_item))
        wheel_com.write(pathpalning.aferCatchToback() + '1O')
        print "finishing putting wheel"
        isComplete(wheel_com)
        arm_com.write(robotarm.Standardpose())




    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), map.getStartpoint())
    wheel_com.write(pathlist + '1q')












__author__ ='Nick chen'


#2018.3.18 19:08 edited by nick chen



import time
import platform
import serial
from pathplanning.Pathplanning import PathPlanning
from maphandle.Map import  Map
from Shelfhandle.Shelf import Shelf
from warehouse.Warehouse import WareHouse
from robotarm.Robotarm import RobotArm


if __name__=='__main__':
    map=Map()
    shelf=Shelf('left')
    warehouse=WareHouse()
    robotarm = RobotArm()
    pathpalning=PathPlanning(map.getMap(),map.getStartpoint(),'N')
    print('Done Initialize Robot.')

    order_capture = map.getSearch_List("area")
    for c_area in order_capture:
        pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), map.getCaptureSite(c_area)[0])
        facelist = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(), "Capture")
        print "the car is go to %s to capture"%c_area

    order_capture.reverse()
    for c_area in order_capture:
        while(True):
            if shelf.getFirst_Uncheckshelf(c_area) ==  None:
                break
            print "shelf.getFirst_Uncheckshelf:",shelf.getFirst_Uncheckshelf(c_area)
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), shelf.getFirst_Uncheckshelf(c_area))
            shelf.CheckShelf(map,pathpalning.getCurrentposition(),c_area)
            print "the car is check the shelf of %s"%c_area


    #after recognizing item_list:
    item_list = {"A": {"left": "red cube", "mid": "shuang wai wai", "right": "apple"},
                 "B": {"left": "green cube", "mid": "blue cube", "right": "tennis ball"},
                 "C": {"left": "xue bi", "mid": "shuang wai wai", "right": "steel ball"},
                 "D": {"left": "yang le duo", "mid": "badminton", "right": "yellow cube"}}


    special_position = ""
    special_area = ""
    special_item=""


    for area in map.getSearch_List("area"):
        item_pos = item_list[area]
        robotarm.initCatchOrder()
        for pos in map.getSearch_List("cargo"):
            item = item_pos[pos]
            if item is None:
                robotarm.delCatchOrder(pos)
                continue

            if item is "apple":
                special_area = area
                special_position = pos
                special_item=item
                robotarm.delCatchOrder(pos)
                continue

            # go to the catching point
            print '-----------------------start-------------------------------------------'
            destination, Existpos = warehouse.getFirst_Existwarehouse(map, area, robotarm.getCatch_Order())
            print 'the coordinate of the first existwarehouse is:%s,position is:%s' % (destination, Existpos)
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), destination)
            print 'the car will go to the destination... order:%s' % (pathlist)
            catchpath = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(),"Catch")
            frontMove = pathpalning.afterTurnFront()
            print 'the car will face to the warehouse... order:%s' % (str(catchpath) + str(frontMove))

            # begin to catch
            send_list = robotarm.ArmMoveto(Existpos) + robotarm.ArmCatch(robotarm.getItemangle(item))
            print "the robotarm will move to catch... order:%s" % send_list
            print "has the robotarm already caught?:%s" % warehouse.catch_warehouse(area, Existpos)
            print "the robotarm is initializing... order:%s" % robotarm.Standardpose()

            # out of catching point
            print "the car will go back from the warehouse... order:%s" % pathpalning.aferCatchToback()

            # go to the putting point
            point ,level = shelf.getFirst_Emptyshelf(shelf.whereIsItem(item))
            print "the coordinate of the first emptyshelf is %s, position is %s" % (point, level)
            pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), point)
            print "the car will  go to the destination... order:%s" % (pathlist)

            # ready to put
            putpath = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(), "Put")
            print "the car is face to the shelf... order:%s"%putpath

            # arm to get up/down
            print "the robotarm is preparing to put... order:%s" % robotarm.ArmMoveto(level)

            # car go forward
            print 'the car will face to the shelf... order:%s' % pathpalning.afterTurnFront()

            # finishing putting
            print "the robotarm will put %s order:%s" % (item,robotarm.ArmEase())
            print "has the robotarm already put?:%s" % shelf.putShelf(map, pathpalning.getCurrentposition(),shelf.whereIsItem(item), level)


            print "get the list of the shelf: %s,area is :%s " % (str(shelf.get_shelf(shelf.whereIsItem(item))), shelf.whereIsItem(item))

            print "the car will go back from the shelf... order:%s" % pathpalning.aferCatchToback()
            print "the robotarm is initializing... order:%s" % robotarm.Standardpose()
            print '-----------------------end-------------------------------------------'




    # go to the catching point
    print '-----------------------start-------------------------------------------'
    destination, Existpos = warehouse.getFirst_Existwarehouse(map, special_area, special_position)
    print 'the coordinate of the first existwarehouse is:%s,position is:%s' % (destination, Existpos)
    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), destination)
    print 'the car will go to the destination... order:%s' % (pathlist)
    catchpath = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(), "Catch")
    frontMove = pathpalning.afterTurnFront()
    print 'the car will face to the warehouse... order:%s' % (str(catchpath) + str(frontMove))

    # begin to catch
    send_list = robotarm.ArmMoveto(Existpos) + robotarm.ArmCatch(robotarm.getItemangle(special_item))
    print "the robotarm will move to catch... order:%s" % send_list
    print "has the robotarm already caught?:%s" % warehouse.catch_warehouse(special_area, Existpos)
    print "the robotarm is initializing... order:%s" % robotarm.Standardpose()

    # out of catching point
    print "the car will go back from the warehouse... order:%s" % pathpalning.aferCatchToback()

    # go to the putting point
    point, level = shelf.getFirst_Emptyshelf(shelf.whereIsItem(special_item))
    print "the coordinate of the first emptyshelf is %s, position is %s" % (point, level)
    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), point)
    print "the car will  go to the destination... order:%s" % (pathlist)

    # ready to put
    putpath = pathpalning.TurnToFace(map, pathpalning.getCurrentposition(), "Put")
    print "the car is face to the shelf... order:%s" % putpath

    # arm to get up/down
    print "the robotarm is preparing to put... order:%s" % robotarm.ArmMoveto(level)

    # car go forward
    print 'the car will face to the shelf... order:%s' % pathpalning.afterTurnFront()

    # finishing putting
    print "the robotarm will put %s order:%s" % (special_item, robotarm.ArmEase())
    print "has the robotarm already put?:%s" % shelf.putShelf(map, pathpalning.getCurrentposition(),
                                                              shelf.whereIsItem(special_item), level)

    print "get the list of the shelf: %s,area is :%s " % (
    str(shelf.get_shelf(shelf.whereIsItem(special_item))), shelf.whereIsItem(special_item))

    print "the car will go back from the shelf... order:%s" % pathpalning.aferCatchToback()
    print "the robotarm is initializing... order:%s" % robotarm.Standardpose()
    print '-----------------------end-------------------------------------------'


    pathlist, count = pathpalning.Goto(pathpalning.getCurrentposition(), map.getStartpoint())
    print "the car will go back to home... order:%s" %pathlist










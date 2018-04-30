__author__ ='Nick chen'

# coding:utf8
from heapq import *
import numpy as np


class PathPlanning(object):

    def __init__(self,Map,current_position,direction):
        self.__map=Map
        self.__current_position=list(current_position)
        self.__direction=direction
        self.__order_ForwardStep = 'F'
        self.__order_BackwardStep = 'B'
        self.__order_LeftTurnStep = 'L'
        self.__order_LeftShiftStep = 'l'
        self.__order_RightTurnStep = 'R'
        self.__order_RightShiftStep = 'r'
        self.__order_TurnToNorth = 'q'
        self.__order_TurnToSouth = 'p'
        self.__order_TurnToWest = 'z'
        self.__order_TurnToEast= 'm'
        self.__order_WheelEnable_false = 's'
        self.__order_WheelEnable_true= 'S'
        self.__order_SerialprintlnComplete = 'O'



    def Astar(self, start, goal):
        def heuristic(a, b):
            x1, y1 = a
            x2, y2 = b
            return (x2 - x1) ** 2 + (y2 - y1) ** 2

        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []

        heappush(oheap, (fscore[start], start))

        while oheap:

            current = heappop(oheap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + heuristic(current, neighbor)
                if 0 <= neighbor[0] < self.__map.shape[0]:
                    if 0 <= neighbor[1] < self.__map.shape[1]:
                        if self.__map[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heappush(oheap, (fscore[neighbor], neighbor))

        return False

    def Goto(self, start, end):
        road_list = self.Astar(start,end)
        #print road_list
        road_list.reverse()
        order_list = []
        mark = 0
        standard_x = start[0]
        standard_y = start[1]
        for point_x,point_y in road_list[mark::]:
            # print road_list[mark::]
            standard_x_ = point_x
            standard_y_ = point_y
            if self.__direction is 'N':
                if standard_x_ > standard_x:
                    order_list.append("B")
                    self.__current_position[0] += 1
                elif standard_x_ < standard_x:
                    order_list.append("F")
                    self.__current_position[0] -= 1
                if standard_y_ > standard_y:
                    order_list.append("r")
                    self.__current_position[1] += 1
                elif standard_y_ < standard_y:
                    order_list.append("l")
                    self.__current_position[1] -= 1
                standard_x = standard_x_
                standard_y = standard_y_
                mark += 1

            elif self.__direction is 'S':
                if standard_x_ > standard_x:
                    order_list.append("F")
                    self.__current_position[0] += 1
                elif standard_x_ < standard_x:
                    order_list.append("B")
                    self.__current_position[0] -= 1
                if standard_y_ > standard_y:
                    order_list.append("l")
                    self.__current_position[1] += 1
                elif standard_y_ < standard_y:
                    order_list.append("r")
                    self.__current_position[1] -= 1
                standard_x = standard_x_
                standard_y = standard_y_
                mark += 1

            elif self.__direction is 'W':
                if standard_x_ > standard_x:
                    order_list.append("l")
                    self.__current_position[0] += 1
                elif standard_x_ < standard_x:
                    order_list.append("r")
                    self.__current_position[0] -= 1
                if standard_y_ > standard_y:
                    order_list.append("B")
                    self.__current_position[1] += 1
                elif standard_y_ < standard_y:
                    order_list.append("F")
                    self.__current_position[1] -= 1
                standard_x = standard_x_
                standard_y = standard_y_
                mark += 1

            elif self.__direction is 'E':
                if standard_x_ > standard_x:
                    order_list.append("r")
                    self.__current_position[0] += 1
                elif standard_x_ < standard_x:
                    order_list.append("l")
                    self.__current_position[0] -= 1
                if standard_y_ > standard_y:
                    order_list.append("F")
                    self.__current_position[1] += 1
                elif standard_y_ < standard_y:
                    order_list.append("B")
                    self.__current_position[1] -= 1
                standard_x = standard_x_
                standard_y = standard_y_
                mark += 1






        order_len = len(order_list)
        temp1 = order_list[0]
        flag = 0
        count=0
        result = ""
        for i in range(order_len):
            temp2 = order_list[i]
            if temp1 == temp2:
                flag = flag + 1
                continue
            result = result + str(flag)
            result = result + str(temp1)
            temp1 = temp2
            flag = 1
            count+=1

        result = result + str(flag)
        result = result + str(temp1)
        count+=1
        return result,count



    def getMap(self):
        return self.__map

    def setMap(self, Map):
        self.__map = Map

    def getCurrentposition(self):
        return tuple(self.__current_position)

    def setCurrentposition(self,point):
        self.__current_position=point

    def getDirection(self):
        return self.__direction

    def setDirection(self,direction):
        self.__direction=direction



    def TurnToFace(self,map,area ,where):
        if (where is 'Capture') or (where is 'Catch'):
            if area is 'A':
                self.__direction = 'W'
                result = "1z"
                return "1z"
            elif area is 'B':
                self.__direction = 'S'
                return "1p"

            elif area is 'C':
                self.__direction = 'E'
                result = "1m"
                return "1m"
            elif area is 'D':
                self.__direction = 'N'
                result = "1q"
                return "1q"

        elif where is 'Put':
            if area is 'A':
                self.__direction='E'
                result = "1m"
                return "1m"

            elif area is 'B':
                self.__direction='N'
                result =  "1q"
                return "1q"

            elif area is 'C':
                self.__direction='W'
                result = "1z"
                return "1z"

            elif area is 'D':
                self.__direction='S'
                result = "1p"
                return "1p"


    def afterTurnFront(self):
        result = "1F"
        return "1F"
    
    def afterTurnSlowFront(self):
        result = "1f"
        return "1f"


    def aferCatchToback(self):
        result = "1B"
        return "1B"








if __name__=='__main__':
    from maphandle.Map import Map
    from Shelfhandle.Shelf import Shelf
    from warehouse.Warehouse import WareHouse
    from robotarm.Robotarm import RobotArm
    map=Map()
    shelf=Shelf('Left')
    pathplanning = PathPlanning(map.getMap(), map.getStartpoint(), 'N')

    
    wareshelf=WareHouse()
    robotarm=RobotArm()
    pathplanning=PathPlanning(map.getMap(),map.getStartpoint(),'N')

    pathplanning.Goto(pathplanning.getCurrentposition(),(2,7))
    print pathplanning.getCurrentposition()
    pathplanning.Goto(pathplanning.getCurrentposition(),(8,2))
    print pathplanning.getCurrentposition()
    















# coding:utf8


"""
the warehouse map is that:
            *
         (1) (1) (1)                    0:None
   (1) C｜B   B   B                     1:exist
*  (1) C｜—————
   (1) C｜      ｜A(1)
      —————｜A(1)   *(CAPTURE)
       D   D   D｜A(1)
      (1) (1) (1)
           *

"""
import numpy as np

class Map(object):
    def __init__(self):
        self.__map= np.array([[6, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 6],  # X:0
                              [3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 5],  # X:1
                              [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5],  # X:2
                              [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5],  # X:3
                              [3, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 5],  # X:4
                              [3, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 5],  # X:5
                              [3, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 5],  # X:6
                              [3, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 5],  # X:7
                              [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5],  # X:8
                              [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5],  # X:9
                              [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 5],  # X:10
                              [6, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 6]])  # X:11
                          # Y: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,11

        self.__StartPoint = (9, 8)

        self.__A_putSite=[(5, 9),(6, 9),(7, 9),(8, 9),(9, 9),(10, 9)]
        self.__B_putSite=[(2, 5),(2, 6),(2, 7),(2, 8),(2, 9),(2, 10)]
        self.__C_putSite=[(6, 2),(5, 2),(4, 2),(3, 2),(2, 2), (1, 2)]
        self.__D_putSite=[(9, 6),(9, 5),(9, 4),(9, 3), (9, 2),(9, 1)]


        self.__A_checkSite = [(5, 9), (6, 9), (7, 9), (8, 9), (9, 9), (10, 9)]
        self.__B_checkSite = [(2, 5), (2, 6), (2, 7), (2, 8), (2, 9), (2, 10)]
        self.__C_checkSite = [(6, 2), (5, 2), (4, 2), (3, 2), (2, 2), (1, 2)]
        self.__D_checkSite = [(9, 6), (9, 5), (9, 4), (9, 3), (9, 2), (9, 1)]


        #CORRECT POINT
        self.__A_catchSite = [(6,8)]
        self.__B_catchSite = [(3,6)]
        self.__C_catchSite = [(5,3)]
        self.__D_catchSite = [(8,5)]


        self.__AcaptureSite = [(6, 9)]
        self.__BcaptureSite = [(2, 6)]
        self.__CcaptureSite = [(5, 2)]
        self.__DcaptureSite = [(9, 5)]


        self.__search_order_area=['A','B','C','D']

        self.__search_list_cargo = [ 'Middle','Left','Right']



    def getPutsite(self,area):
        if area is 'A':
            return self.__A_putSite
        elif area is 'B':
            return self.__B_putSite
        elif area is 'C':
            return self.__C_putSite
        elif area is 'D':
            return self.__D_putSite



    def getCatchSite(self,area):
        if area is 'A':
            return self.__A_catchSite

        elif area is 'B':
            return self.__B_catchSite

        elif area is 'C':
            return self.__C_catchSite

        elif area is 'D':
            return self.__D_catchSite


    def getCaptureSite(self,area):
        if area is 'A':
            return self.__AcaptureSite

        elif area is 'B':
            return self.__BcaptureSite

        elif area is 'C':
            return self.__CcaptureSite

        elif area is 'D':
            return self.__DcaptureSite


    def getSite(self,area,what):
        if what is "Capture":
            return self.getCaptureSite(area)

        elif what is "Catch":
            return self.getCatchSite(area)

        elif what is "Put":
            return self.getPutsite(area)

        return None




    def getChecksite(self,area):
        if area is 'A':
            return self.__A_checkSite
        elif area is 'B':
            return self.__B_checkSite
        elif area is 'C':
            return self.__C_checkSite
        elif area is 'D':
            return self.__D_checkSite


    def getStartpoint(self):
        return self.__StartPoint


    def getMap(self):
        return self.__map




    def getSearch_List(self,where):
        if where is "cargo":
            return self.__search_list_cargo
        elif where is "area":
            return self.__search_order_area

if __name__=='__main__':
    map=Map()
    print map.getCatchSite('A')












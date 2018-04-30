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



class WareHouse():
    def __init__(self):
        self.__warehouse_A = [1, 1, 1]
        self.__warehouse_B = [1, 1, 1]
        self.__warehouse_C = [1, 1, 1]
        self.__warehouse_D = [1, 1, 1]
        self.__Itemslist = ("RedCube", "YangLeDuo", "Badminton",         # area A
                            "YellowCube", "ShuangWaiWai", "SteelBall", #area  B
                            "BlueCube", "Snow", "Apple",                 #area  C
                            "GreenCube", "Sprite", "Tennis")          #area  D


    def getwarehouse(self,area):
        if area is 'A':
            return self.__warehouse_A
        elif area is 'B':
            return self.__warehouse_B
        elif area is 'C':
            return self.__warehouse_C
        elif area is 'D':
            return self.__warehouse_D


    def setwarehouse(self,area,index,result):
        if area is 'A':
            self.__warehouse_A[index]=result
        elif area is 'B':
            self.__warehouse_B[index]=result
        elif area is 'C':
            self.__warehouse_C[index]=result
        elif area is 'D':
            self.__warehouse_D[index]=result

    def getItemlist(self):
        return self.__Itemslist


    def isExsist(self,area):
        match={0}
        warehouse=self.getwarehouse(area)
        temp = set(warehouse)
        result = temp | match
        if result == {0}:
            return False

        return True



    def catch_warehouse(self,area,position):
        if position is 'Left':
            self.getwarehouse(area)[0] = 0
            return True
        elif position is 'Middle':
            self.getwarehouse(area)[1] = 0
            return True
        elif position is 'Right':
            self.getwarehouse(area)[2] = 0
            return True
        else:
            print "%s IS NOT CATCH AREA!-----Shelf.py" %area
            return False



    def getFirst_Existwarehouse(self,map,area,catch_order):
        catch_list = {'Left': 0, 'Middle': 1, 'Right': 2}
        if self.getwarehouse(area)[catch_list[catch_order]] == 1:
            return map.getCatchSite(area)[0],catch_order
        return None,None







if __name__=='__main__':
    from maphandle.Map import Map
    area = ['A', 'B', 'C', 'D']
    position=['Left','Middle','Right']
    catch_order = ['Left','Middle','Right']
    map=Map()
    warehouse=WareHouse()
    for a in area:
        print '\n'
        for catch in catch_order:
            print "area:", a
            print "First_Existwarehouse:", warehouse.getFirst_Existwarehouse(map, a, catch_order)
            point, position = warehouse.getFirst_Existwarehouse(map, a, catch_order)
            print "catch_warehouse:", warehouse.catch_warehouse(a, position)
            print "warehouse:", warehouse.getwarehouse(a)
            print "isExsist?", warehouse.isExsist(a)
            print "-----------------------------------"






















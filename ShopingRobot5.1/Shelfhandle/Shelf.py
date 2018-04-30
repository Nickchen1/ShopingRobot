# coding:utf8

"""

the shelf map is that:

     ---- ---- ---- ---- ---- ----
    £ü 0 £ü 0 £ü 0 £ü 0 £ü 0 £ü 0 £ü
     ---- ---- ---- ---- ---- ----
    £ü 0 £ü 0 £ü 0 £ü 0 £ü 0 £ü 0 £ü
     ---- ---- ---- ---- ---- ----

"""


class Shelf(object):
    def  __init__(self,direction):
        self.__cargo_A = [[0, 0, 0, 0, 0 ,0],
                          [0, 0, 0, 0, 0 ,0]]
        self.__cargo_A_check = [0, 0, 0, 0, 0, 0]

        self.__cargo_B = [[0, 0, 0, 0, 0 ,0],
                          [0, 0, 0, 0, 0 ,0]]
        self.__cargo_B_check = [0, 0, 0, 0, 0, 0]

        self.__cargo_C = [[0, 0, 0, 0, 0 ,0],
                          [0, 0, 0, 0, 0 ,0]]
        self.__cargo_C_check = [0, 0, 0, 0, 0, 0]

        self.__cargo_D = [[0, 0, 0, 0, 0 ,0],
                          [0, 0, 0, 0, 0 ,0]]
        self.__cargo_D_check = [0, 0, 0, 0, 0, 0]

        self.__dir=direction

        self.__ShlefA = ("RedCube", "YangLeDuo", "Badminton")
        self.__ShlefB = ("YellowCube", "ShuangWaiWai", "SteelBall")
        self.__ShlefC = ("BlueCube", "Snow", "Apple")
        self.__ShlefD = ("GreenCube", "Sprite", "Tennis")

        if direction is 'Left':
            self.__k_A_B = 1
            self.__b_A_B = 5
            self.__k_C_D = -1
            self.__b_C_D = 6

        elif direction is 'Right':
            self.__k_A_B = -1
            self.__b_A_B = 10
            self.__k_C_D = 1
            self.__b_C_D = 1

    def get_K(self, area):
        if (area is "A") or (area is "B"):
            return self.__k_A_B
        elif (area is "C") or (area is "D"):
            return self.__k_C_D

    def get_B(self, area):
        if (area is "A") or (area is "B"):
            return self.__b_A_B
        elif (area is "C") or (area is "D"):
            return self.__b_C_D



    def getChecklist(self, area):
        if area is "A":
            return self.__cargo_A_check
        elif area is "B":
            return self.__cargo_B_check
        elif area is "C":
            return self.__cargo_C_check
        elif area is "D":
            return self.__cargo_D_check

        print "NONE OF CHECKLIST IN %s" %area
        return None


    def get_shelf(self, area):
        if area is 'A':
            return self.__cargo_A
        elif area is 'B':
            return self.__cargo_B
        elif area is 'C':
            return self.__cargo_C
        elif area is 'D':
            return self.__cargo_D



    def set_shelf(self,area,result):
        if area is 'A':
            self.__cargo_A = result
        elif area is 'B':
            self.__cargo_B = result
        elif area is 'C':
            self.__cargo_C = result
        elif area is 'D':
            self.__cargo_D = result



    def isEmpty_shelf(self,area):
        match={0}
        cargo=self.get_shelf(area)
        temp1 = set(cargo[0])
        temp2 = set(cargo[1])
        result = temp1 | match | temp2
        if result == {1}:
            return False

        return True


    def isUncheck_shelf(self,area):
        match={0}
        check=self.getChecklist(area)
        temp = set(check)
        result = temp | match
        if result == {1}:
            return False

        return True


    def shelfTopoint(self, area, index):
        if area is "A":
            return self.get_K(area) * index + self.get_B(area), 9
        elif area is "B":
            return 2, self.get_K(area) * index + self.get_B(area)
        elif area is "C":
            return self.get_K(area) * index + self.get_B(area), 2
        elif area is "D":
            return 9, self.get_K(area) * index + self.get_B(area)

        return None



    def pointToshelf(self,map,area, point):
        if point in map.getPutsite(area):
            if self.__dir is 'Left':
                if area is "A":
                    p = self.get_K(area) * point[0] - self.get_B(area)
                    return p
                elif area is "B":
                    p = self.get_K(area) * point[1] - self.get_B(area)
                    return p
                elif area is "C":
                    p = self.get_K(area) * point[0] + self.get_B(area)
                    return p
                elif area is "D":
                    p = self.get_K(area) * point[1] + self.get_B(area)
                    return p

            elif self.__dir is 'Right':
                if area is "A":
                    p = self.get_K(area) * point[0] + self.get_B(area)
                    return p
                elif area is "B":
                    p = self.get_K(area) * point[1] + self.get_B(area)
                    return p
                elif area is "C":
                    p = self.get_K(area) * point[0] - self.get_B(area)
                    return p
                elif area is "D":
                    p = self.get_K(area) * point[1] - self.get_B(area)
                    return p
        else:
            print "NONE OF PUTSITE IS %s  ------shelf.py" %area


    def putShelf(self,map,point,area,level):
        if (level is "up") or (level is "all"):
            self.get_shelf(area)[0][self.pointToshelf(map, area, point)] = 1
            return True
        elif level is "low":
            self.get_shelf(area)[1][self.pointToshelf(map,area, point)] = 1
            return True
        else:
            print "NONE OF LEVEL IS %s" %level
            return False



    def getFirst_Emptyshelf(self,area):
        upshelf = self.get_shelf(area)[0]
        lowshelf = self.get_shelf(area)[1]
        for index, shelf in enumerate(zip(upshelf, lowshelf)):
            if shelf[0] == 1 and shelf[1] == 0:
                return self.shelfTopoint(area, index), 'low'

            elif shelf[0] == 0 and shelf[1] == 1:
                return self.shelfTopoint(area, index), 'up'

            elif shelf[0] == 0 and shelf[1] == 0:
                return self.shelfTopoint(area, index), 'all'

        print "NONE OF EMPTY  OF SHELF!-----Shelf.py "
        return None



    def iScheckShelf(self, map, area, point):
        status = self.getChecklist(area)[self.pointToshelf(map, area, point)]
        if status == 1:
            return True

        return False

    def CheckShelf(self, map, point, area):
        p = self.pointToshelf(map, area, point)
        self.getChecklist(area)[p] = 1
        return True


    def getFirst_Uncheckshelf(self,area):
        uncheckshelf = self.getChecklist(area)
        print "uncheckshelf:",uncheckshelf
        for index,uncheck in enumerate(uncheckshelf):
            if self.isUncheck_shelf(area):
                if uncheck is 0:
                    return self.shelfTopoint(area, index)
            else:
                return None

        return None




    def whereIsItem(self,item):
        if item in self.__ShlefA:
            return "A"
        elif item in self.__ShlefB:
            return "B"
        elif item in self.__ShlefC:
            return "C"
        elif item in self.__ShlefD:
            return "D"



if __name__=='__main__':
    from maphandle.Map import Map
    from warehouse.Warehouse import WareHouse
    area=['A']
    dir=['Left']
    map=Map()
    warehouse = WareHouse()
    count=0
    for d in dir:
        shelf = Shelf(d)
        print '\n'
        for a in area:
            print '\n'
            point,level = shelf.getFirst_Emptyshelf(a)
            print "First_Emptyshelf:", point, level
            print "getPutsite:", point
            print "putShelf:", shelf.putShelf(map, point, a, level)
            print "area:", a
            print 'level:', level
            print 'dir:', d
            print "get_shelf:", shelf.get_shelf(a)
            print "-----------------------------------"




















# coding:utf8

class RobotArm(object):
    def __init__(self):

        self.__catch_order=['Left','Middle','Right']


        self.__Initangle = {'X': -44.0, 'Y': -45.0, 'Z': 0.0}

        self.__CatchleftAngle = {'X':-44.0,'Y':0.0,'Z':33.0}
        self.__CatchmidAngle = {'X':-39.0,'Y':-7.0,'Z':0.0}
        self.__CatchrightAngle = {'X':-42.0,'Y':0.0,'Z':-32.0}

        self.__Putupangle = {'X':0.0,'Y':40.5,'Z':0.0}
        self.__Putlowangle = {'X':-33.0,'Y':0.0,'Z':0.0}

        self.__Itemangle = {"RedCube":64 ,"YangLeDuo":75,"Badminton":80,
                          "YellowCube":64 ,"ShuangWaiWai":64 ,"SteelBall":70,
                          "BlueCube":64 ,"Snow":60,"Apple":62,
                          "GreenCube":64,"Sprite":60,"Tennis":65 }

        self.__Captureangle = {'X': -44.0, 'Y': -10.0, 'Z': 0.0 }

        self.__ShelfCheckangle = {'X': -46.0, 'Y': -41.0, 'Z': 0.0 }



    def armInit(self):
        return 'M17\nG28\nG95\nG1 X%s Y%s\nG95\nG93 X0.0 Y0.0 Z0.0\n' %(str(self.__Initangle['X']),str(self.__Initangle['Y']))

    def ArmMoveto(self,position):
        if position is 'Middle':
            command = 'G95\nG1 X%s Y%s Z%s\n' %(str(self.__CatchmidAngle['X']),str(self.__CatchmidAngle['Z']) , str(self.__CatchmidAngle['Z']))
            return 'G95\nG1 X%s Y%s Z%s\n' %(str(self.__CatchmidAngle['X']),str(self.__CatchmidAngle['Z']) , str(self.__CatchmidAngle['Z']))

        elif position is 'Left':
            command = 'G95\nG1 Z%s\nG1 X%s Y%s\n' %(str(self.__CatchleftAngle['Z']),str(self.__CatchleftAngle['X']),str(self.__CatchleftAngle['Y']))
            return 'G95\nG1 Z%s\nG1 X%s Y%s\n' %(str(self.__CatchleftAngle['Z']),str(self.__CatchleftAngle['X']),str(self.__CatchleftAngle['Y']))

        elif position is 'Right':
            command = 'G95\nG1 Z%s\nG1 X%s Y%s\n' % (str(self.__CatchrightAngle['Z']), str(self.__CatchrightAngle['X']), str(self.__CatchrightAngle['Y']))
            return 'G95\nG1 Z%s\nG1 X%s Y%s\n' % (str(self.__CatchrightAngle['Z']), str(self.__CatchrightAngle['X']), str(self.__CatchrightAngle['Y']))

        elif position is 'low':
            command = 'G95\nG1 X%s Y%s Z%s\n' % (str(self.__Putlowangle['X']), str(self.__Putlowangle['Y']), str(self.__Putlowangle['Z']))
            return 'G95\nG1 X%s Y%s Z%s\n' % (str(self.__Putlowangle['X']), str(self.__Putlowangle['Y']), str(self.__Putlowangle['Z']))

        elif (position is "up") or (position  is "all"):
            command = 'G95\nG1 X%s Y%s Z%s\n' % (str(self.__Putupangle['X']), str(self.__Putupangle['Y']), str(self.__Putupangle['Z']))
            return 'G95\nG1 X%s Y%s Z%s\n' % (str(self.__Putupangle['X']), str(self.__Putupangle['Y']), str(self.__Putupangle['Z']))

        elif position is 'capture':
            command = 'G95\nG1 X%s Y%s Z%s\n' % (str(self.__Captureangle['X']), str(self.__Captureangle['Y']), str(self.__Captureangle['Z']))
            return 'G95\nG1 X%s Y%s Z%s\n' % (str(self.__Captureangle['X']), str(self.__Captureangle['Y']), str(self.__Captureangle['Z']))



    def ArmCatch(self,item):
        result = 'M280 P0 S%s\n' %str(item)
        return 'M280 P0 S%s\n' %str(item)


    def ArmEase(self):
        result =  'M280 P0 S15\n'
        return 'M280 P0 S15\n'


    def Standardpose(self):
        command = 'G90\nG95\nG1 X0.0 Y0.0\nG1 Z0.0\n'
        return 'G95\nG1 X0.0 Y0.0\nG1 Z0.0\n'


    def getItemangle(self,item):
        result = self.__Itemangle[item]
        return result

    def getCatch_Order(self):
        result = self.__catch_order
        return result

    def delCatchOrder(self, pos):
        self.__catch_order.remove(pos)

    def initCatchOrder(self):
        self.__catch_order = ['Left','Middle','Right']

if __name__=='__main__':
    from warehouse.Warehouse import WareHouse
    robotarm = RobotArm()
    print robotarm.ArmMoveto("capture")
    warehouse = WareHouse()
    command=['Left','Middle','Right','low','up']
    itemlist=warehouse.getItemlist()
    for position in command:
        for item in itemlist:
            print "robotarm.armInit():",robotarm.armInit()
            print "robotarm.ArmMoveto "+ position +":",robotarm.ArmMoveto(position)
            print "robotarm.ArmCatch "+ item + ":",robotarm.ArmCatch(robotarm.getItemangle(item))
            print "robotarm.ArmEase():",robotarm.ArmEase()
            print "robotarm.Standardpose():",robotarm.Standardpose()
            print "-----------------------------------"














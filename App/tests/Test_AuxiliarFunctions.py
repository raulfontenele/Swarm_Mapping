import unittest

from Source.AuxiliarFunctions import AuxiliarFunctions
#from AuxiliarFunctions import AuxiliarFunctions


class Test_AuxiliarFunctions(unittest.TestCase):
    def test_DiffAngleThreshold(self):
        ## Quadrant 1
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,54),54)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(50,0),-50)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,90),90)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(90,0),-90)
        '''
        ## Quadrant 2
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(110,158),48)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(150,110),-40)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(93,186),93)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(180,93),-87)

        ## Quadrant 3
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(190,225),35)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(225,194),-31)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(203,270),67)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(265,193),-72)

        ## Quadrant 4
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(270,350),80)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(360,321),-39)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(285,314),29)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(354,360),6)

        ## 1º - > 2º -> 1º
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(20,100),80)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(100,20),-80)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,180),180)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(180,0),-180)

        ## 1º - > 3º -> 1º
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,270),-90)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(270,0),90)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(15,220),155)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(220,0),140)

        ## 1º - > 4º -> 1º
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,350),10)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(330,20),50)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(280,5),85)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(24,251),33)

        ## Same value
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,360),0)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(360,0),0)
        '''
    def test_AngleDistance(self):
        # 1º Quadrant
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([0,0],[10,-10]),(14.142,45))
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([0,0],[20,0]),(20,0))

        # 2º Quadrant
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([0,0],[-10,-10]),(14.142,45+90))
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([5,5],[-15,-15]),(28.284,45+90))
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([-3,-5],[-3,-55]),(50,90))

        # 3º Quadrant
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([2,5],[-3,20]),(15.811,251.565))
        
        # 4º Quadrant
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([0,0],[10,0]),(10,0))
        self.assertEqual(AuxiliarFunctions.CalcAngleDistance([5,5],[-15,5]),(20,180))


    def test_AngleDiff(self):
        # 1º Quadrant Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(0,10,1),10)
        self.assertEqual(AuxiliarFunctions.diffAngles(13,27,1),14)
        self.assertEqual(AuxiliarFunctions.diffAngles(0,90,1),90)

        # 1º Quadrant Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(0,10,-1),350)
        self.assertEqual(AuxiliarFunctions.diffAngles(13,27,-1),346)
        self.assertEqual(AuxiliarFunctions.diffAngles(0,90,-1),270)

        # 2º Quadrant Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(91,180,1),89)
        self.assertEqual(AuxiliarFunctions.diffAngles(123,175,1),52)
        self.assertEqual(AuxiliarFunctions.diffAngles(137,175,1),38)

        # 2º Quadrant Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(91,180,-1),271)
        self.assertEqual(AuxiliarFunctions.diffAngles(123,175,-1),308)
        self.assertEqual(AuxiliarFunctions.diffAngles(137,175,-1),322)

        # 3º Quadrant Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(180,270,1),90)
        self.assertEqual(AuxiliarFunctions.diffAngles(123+90,175+90,1),52)
        self.assertEqual(AuxiliarFunctions.diffAngles(137+90,175+90,1),38)

        # 3º Quadrant Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(180,270,-1),270)
        self.assertEqual(AuxiliarFunctions.diffAngles(123+90,175+90,-1),308)
        self.assertEqual(AuxiliarFunctions.diffAngles(137+90,175+90,-1),322)
        
        # 4º Quadrant Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(270,360,1),90)
        self.assertEqual(AuxiliarFunctions.diffAngles(123+180,175+180,1),52)
        self.assertEqual(AuxiliarFunctions.diffAngles(137+180,175+180,1),38)

        # 4º Quadrant Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(270,360,-1),270)
        self.assertEqual(AuxiliarFunctions.diffAngles(123+180,175+180,-1),308)
        self.assertEqual(AuxiliarFunctions.diffAngles(137+180,175+180,-1),322)

        # Between 1 and 2 Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(20,163,1),143)
        self.assertEqual(AuxiliarFunctions.diffAngles(0,180,1),180)
        self.assertEqual(AuxiliarFunctions.diffAngles(56,176,1),120)

        # Between 1 and 2 Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(20,163,-1),217)
        self.assertEqual(AuxiliarFunctions.diffAngles(0,180,-1),180)
        self.assertEqual(AuxiliarFunctions.diffAngles(56,176,-1),240)

        # Between 2 and 3 Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(120,263,1),143)
        self.assertEqual(AuxiliarFunctions.diffAngles(90,270,1),180)
        self.assertEqual(AuxiliarFunctions.diffAngles(156,266,1),110)

        # Between 2 and 3 Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(120,263,-1),217)
        self.assertEqual(AuxiliarFunctions.diffAngles(90,270,-1),180)
        self.assertEqual(AuxiliarFunctions.diffAngles(156,266,-1),250)

        # Between 3 and 4 Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(220,325,1),105)
        self.assertEqual(AuxiliarFunctions.diffAngles(180,360,1),180)
        self.assertEqual(AuxiliarFunctions.diffAngles(256,346,1),90)

        # Between 3 and 4 Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(220,325,-1),255)
        self.assertEqual(AuxiliarFunctions.diffAngles(180,360,-1),180)
        self.assertEqual(AuxiliarFunctions.diffAngles(256,346,-1),270)

        # Between 4 and 1 Clockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(350,20,1),30)
        self.assertEqual(AuxiliarFunctions.diffAngles(270,360,1),90)
        self.assertEqual(AuxiliarFunctions.diffAngles(326,21,1),55)
        self.assertEqual(AuxiliarFunctions.diffAngles(270,0,1),90)

        # Between 4 and 1 Anticlockwise
        self.assertEqual(AuxiliarFunctions.diffAngles(350,20,-1),330)
        self.assertEqual(AuxiliarFunctions.diffAngles(270,360,-1),270)
        self.assertEqual(AuxiliarFunctions.diffAngles(326,21,-1),305)
        self.assertEqual(AuxiliarFunctions.diffAngles(270,0,-1),270)

        # Same point
        #self.assertEqual(AuxiliarFunctions.diffAngles(0,360,1),0)
        self.assertEqual(AuxiliarFunctions.diffAngles(0,360,-1),0)

        self.assertEqual(AuxiliarFunctions.diffAngles(90,90,1),0)
        self.assertEqual(AuxiliarFunctions.diffAngles(90,90,-1),0)

        self.assertEqual(AuxiliarFunctions.diffAngles(270,270,1),0)
        self.assertEqual(AuxiliarFunctions.diffAngles(270,270,-1),0)



if __name__ == '__name__':
    unittest.main()

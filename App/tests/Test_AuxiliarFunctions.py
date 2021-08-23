import unittest

from Source.AuxiliarFunctions import AuxiliarFunctions
#from AuxiliarFunctions import AuxiliarFunctions


class Test_AuxiliarFunctions(unittest.TestCase):
    def test_DiffAngle(self):
        ## Quadrant 1
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,54),54)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(50,0),-50)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(0,90),90)
        self.assertEqual(AuxiliarFunctions.diffAngleThreshold(90,0),-90)

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



if __name__ == '__name__':
    unittest.main()

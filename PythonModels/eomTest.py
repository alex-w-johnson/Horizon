import unittest
import sys
import clr
import System.Collections.Generic
import System
clr.AddReference('System.Core')
clr.AddReference('IronPython')
clr.AddReference('System.Xml')
clr.AddReferenceByName('Utilities')
clr.AddReferenceByName('HSFUniverse')
clr.AddReferenceByName('Horizon')
clr.AddReferenceByName('MissionElements')
clr.AddReferenceByName('UserModel')
import Utilities
import HSFUniverse
import math
import MissionElements
import UserModel
from UserModel import XmlParser
from MissionElements import Asset
import eomSSTN
import Horizon
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0
from System import Array
from System import Xml
import datetime #For using WMM
from System import DateTime #For using WMM

class BuildNewEOMS(unittest.TestCase):
    def test_new(self):
        node = XmlParser.GetModelNode(AppDomain.CurrentDomain.BaseDirectory+"\\PythonModels\\eomTest.py")
        testEoms = eomSSTN(eomSSTN,node)
        assert testEoms is not None
if __name__ == '__main__':
    unittest.main()

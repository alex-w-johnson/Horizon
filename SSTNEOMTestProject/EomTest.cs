using NUnit.Framework;
using HSFSystem;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using MissionElements;
using UserModel;
using Horizon;
using System.IO;
using Utilities;
using PythonModels

namespace SSTNEOMTestProject
{
    [TestClass]
    public class EomTest
    {
        [SetUp]
        public void Init()
        {
            XMLNode node = XmlParser.GetModelNode("C:\\Users\\Alex\\Source\\Repos\\alex-w-johnson\\Horizon\\PythonModels\\eomTest.py");
            testEoms = new eomSSTN(Utilities.EOMS, node);
        }
    }
}

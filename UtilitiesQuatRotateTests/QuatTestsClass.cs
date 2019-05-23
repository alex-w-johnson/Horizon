using Microsoft.VisualStudio.TestTools.UnitTesting;
using Utilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Utilities.Tests
{
    [TestClass()]
    public class QuatTestsClass
    {
        readonly Quat testQuat = new Quat(1.0, 0.0, 0.0, 0.0);
        readonly Vector testVector = new Vector("[1.0, 0.0, 0.0]");

        [TestMethod()]
        public void RotateTestMethod()
        {
            Assert.Fail();
        }
    }
}
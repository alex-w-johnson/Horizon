using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Horizon;
using MissionElements;
using Utilities;
using System.Xml;

namespace EOMS_Test
{
    [TestClass]
    public class EomsTests
    {
        Program program;

        [TestMethod]
        public void Init()
        {
            program = new Program();
            string[] inputArg = { "-m", @"C:\\Users\\Alex\\source\\repos\\alex-w-johnson\\Horizon\\CAN_Model_EOMSTest.xml", "-s", @"C:\\Users\\Alex\\source\\repos\\alex-w-johnson\\Horizon\\SimulationInput_CAN.xml", "-t", @"C:\\Users\\Alex\\source\\repos\\alex-w-johnson\\Horizon\\SSTN_Targets_Null.xml" };
            program.InitInput(inputArg);
            string outputPath = program.InitOutput();
            Stack<Task> systemTasks = program.LoadTargets();
            program.LoadSubsystems();
            program.LoadDependencies();
            program.CreateSchedules(systemTasks);
        }

        public void TestMethod1()
        {

        }
    }
}

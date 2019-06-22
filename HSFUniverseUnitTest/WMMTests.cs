using NUnit.Framework;
using HSFUniverse;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;

namespace HSFUniverse.Tests
{
    [TestFixture()]
    public class WMMTests
    {
        [Test()]
        [TestCaseSource(typeof(WMMData), "TestCases")]
        public void GeoMagTest(DateTime date, double alt, double lat, double lon, double bx, double by, double bz, double inc, double dec)
        {
            WMM gm = new WMM();
            gm.GeoMag(lat, lon, alt, date);
            Assert.Multiple(() =>
            {
                Assert.That(() => gm.bx, Is.EqualTo(bx).Within(1));
                Assert.That(() => gm.by, Is.EqualTo(by).Within(1));
                Assert.That(() => gm.bz, Is.EqualTo(bz).Within(1));
                Assert.That(() => gm.dec, Is.EqualTo(dec).Within(0.1));
                Assert.That(() => gm.dip, Is.EqualTo(inc).Within(0.1));
            });
        }
    }
    public class WMMData
    {
        public static IEnumerable TestCases
        {
            get
            { 
                yield return new TestCaseData(new DateTime(2015, 1, 1), 0, 80, 0, 6636.6, -451.9, 54408.9, 83.03, -3.90);
                yield return new TestCaseData(new DateTime(2015, 1, 1), 0, 0, 120, 39521.1, 377.7, -11228.8, -15.86, 0.55);
                yield return new TestCaseData(new DateTime(2015, 1, 1), 0, -80, 240, 5796.3, 15759.1, -52927.1, -72.40, 69.81);
                yield return new TestCaseData(new DateTime(2015, 1, 1), 100, 80, 0, 6323.4, -477.6, 52249.1, 83.08, -4.32);
                yield return new TestCaseData(new DateTime(2015, 1, 1), 100, 0, 120, 37538.1, 351.1, -10751.1, -15.98, 0.54);
                yield return new TestCaseData(new DateTime(2015, 1, 1), 100, -80, 240, 5612.2, 14789.3, -50385.8, -72.57, 69.22);
                yield return new TestCaseData(new DateTime(2017, 7, 2), 0, 80, 0, 6605.2, -298.7, 54506.3, 83.08, -2.59);
                yield return new TestCaseData(new DateTime(2017, 7, 2), 0, 0, 120, 39569.4, 252.3, -11067.9, -15.63, 0.37);
                yield return new TestCaseData(new DateTime(2017, 7, 2), 0, -80, 240, 5864.6, 15764.1, -52706.1, -72.30, 69.59);
                yield return new TestCaseData(new DateTime(2017, 7, 2), 100, 80, 0, 6294.3, -331.1, 52337.8, 83.13, -3.01);
                yield return new TestCaseData(new DateTime(2017, 7, 2), 100, 0, 120, 37584.4, 235.7, -10600.5, -15.75, 0.36);
                yield return new TestCaseData(new DateTime(2017, 7, 2), 100, -80, 240, 5674.9, 14793.1, -50179.5, -72.48, 69.01);
            }
        }
    }
}
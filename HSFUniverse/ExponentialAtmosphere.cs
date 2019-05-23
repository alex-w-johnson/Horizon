using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HSFUniverse
{
    public class ExponentialAtmosphere : Atmosphere
    {
        #region Attributes
        SortedList<double, double[]> lookUpTable = new SortedList<double, double[]>();
        protected const double EARTH_RADIUS = 6378.137;
        #endregion

        #region Constructors
        /// <summary>
        /// Implementation of Vallado Exponential Atmosphere [Vallado Table 8-4]
        /// </summary>
        public ExponentialAtmosphere()
        {
            CreateAtmosphere();
        }
        #endregion

        #region Methods
        /// <summary>
        /// Creates reference lookup table for calculating atmosphere values.
        /// All altitude values are in kilometers
        /// </summary>
        public override void CreateAtmosphere()
        {
            lookUpTable.Add(0, (new double[] { 1.225, 7.249 }));
            lookUpTable.Add(25, (new double[] { 3.899E-2, 6.349 }));
            lookUpTable.Add(30, (new double[] { 1.774E-2, 6.682 }));
            lookUpTable.Add(40, (new double[] { 3.972E-3, 7.554 }));
            lookUpTable.Add(50, (new double[] { 1.057E-3, 8.382 }));
            lookUpTable.Add(60, (new double[] { 3.206E-4, 7.714 }));
            lookUpTable.Add(70, (new double[] { 8.770E-5, 6.549 }));
            lookUpTable.Add(80, (new double[] { 1.905E-5, 5.799 }));
            lookUpTable.Add(90, (new double[] { 3.396E-6, 5.382 }));
            lookUpTable.Add(100, (new double[] { 5.297E-7, 5.877 }));
            lookUpTable.Add(110, (new double[] { 9.661E-8, 7.263 }));
            lookUpTable.Add(120, (new double[] { 2.438E-8, 9.473 }));
            lookUpTable.Add(130, (new double[] { 8.484E-9, 12.636 }));
            lookUpTable.Add(140, (new double[] { 3.845E-9, 16.149 }));
            lookUpTable.Add(150, (new double[] { 2.070E-9, 22.523 }));
            lookUpTable.Add(180, (new double[] { 5.464E-10, 29.740 }));
            lookUpTable.Add(200, (new double[] { 2.789E-10, 37.105 }));
            lookUpTable.Add(250, (new double[] { 7.248E-11, 45.546 }));
            lookUpTable.Add(300, (new double[] { 2.418E-11, 53.628 }));
            lookUpTable.Add(350, (new double[] { 9.518E-12, 53.298 }));
            lookUpTable.Add(400, (new double[] { 3.725E-12, 58.515 }));
            lookUpTable.Add(450, (new double[] { 1.585E-12, 60.828 }));
            lookUpTable.Add(500, (new double[] { 6.967E-13, 63.822 }));
            lookUpTable.Add(600, (new double[] { 1.454E-13, 71.835 }));
            lookUpTable.Add(700, (new double[] { 3.614E-14, 88.667 }));
            lookUpTable.Add(800, (new double[] { 1.170E-14, 124.64 }));
            lookUpTable.Add(900, (new double[] { 5.245E-15, 181.05 }));
            lookUpTable.Add(1000, (new double[] { 3.019E-15, 268.00 }));
        }

        public override double density(double height)
        {
            if (height >= lookUpTable.Last().Key)
            {
                double keymax = lookUpTable.Last().Key;
                return lookUpTable[keymax].ElementAt(0) * Math.Exp(-1.0*(height-keymax)/ lookUpTable[keymax].ElementAt(1));
            }
            else if (height >= lookUpTable.First().Key)
            {
                double key = lookUpTable.TakeWhile(x => x.Key <= height).Last().Key;
                return lookUpTable[key].ElementAt(0) * Math.Exp(-1.0 * (height - key) / lookUpTable[key].ElementAt(1));
            }
            else
            {
                throw new ArgumentOutOfRangeException("height","Altitude must be above surface of Earth");
            }

        }

        public override double pressure(double height)
        {
            throw new NotImplementedException();
        }
        public override double temperature(double height)
        {
            throw new NotImplementedException();
        }
        public override double uVelocity(double height)
        {
            throw new NotImplementedException();
        }
        public override double vVelocity(double height)
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}

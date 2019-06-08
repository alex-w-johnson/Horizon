// Copyright (c) 2016 California Polytechnic State University
// Authors: Morgan Yost (morgan.yost125@gmail.com) Eric A. Mehiel (emehiel@calpoly.edu)

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Utilities
{
    [Serializable]
    public class Quat
    {
        public double _eta { get; set; }
        public Vector _eps { get; set; }

        public Quat()
        {
            _eta = 1;
            _eps = new Vector(3);
        }
        public Quat(double eta, Vector eps)
        {
            _eta = eta;
            _eps = eps;
        }
        public Quat(double eta, double eps1, double eps2, double eps3)
        {
            _eta = eta;
            _eps = new Vector(3);
            _eps[1] = eps1;
            _eps[2] = eps2;
            _eps[3] = eps3;
        }

        public static Quat Conjugate(Quat q)
        {
            Quat p = new Quat();
            {
                p._eta = q._eta;
                p._eps = -1 * q._eps;
            }
            return p;
        }

        public static Vector Rotate(Quat q, Vector a)
        {
            Matrix<double> c1 = new Matrix<double>(3, 1);
            Matrix<double> c2 = new Matrix<double>(3, 1);
            Matrix<double> c3 = new Matrix<double>(3, 1);
            Vector b = new Vector(3);
            c1 = (2 * (q._eta * q._eta) - 1.0) * Matrix<double>.Eye(3);
            Matrix<double> epsilon = new Matrix<double>(q._eps.ToString());
            Matrix<double> quatEps = Matrix<double>.Transpose(epsilon);
            c2 = 2 * quatEps * Matrix<double>.Transpose(quatEps);
            c3 = 2 * q._eta * Matrix<double>.CrossMatrix(q._eps);
            b = (c1 + c2 + c3) * a;
            return b;
        }

        public static Matrix<double> Rotate(Quat q, Matrix<double> a)
        {
            Matrix<double> c1 = new Matrix<double>(3, 1);
            Matrix<double> c2 = new Matrix<double>(3, 1);
            Matrix<double> c3 = new Matrix<double>(3, 1);
            Matrix<double> b = new Matrix<double>(3, 1);
            c1 = (2 * (q._eta * q._eta) - 1.0) * Matrix<double>.Eye(3);
            Matrix<double> epsilon = new Matrix<double>(q._eps.ToString());
            Matrix<double> quatEps = Matrix<double>.Transpose(epsilon);
            c2 = 2 * quatEps * Matrix<double>.Transpose(quatEps);
            c3 = 2 * q._eta * Matrix<double>.CrossMatrix(q._eps);
            b = (c1 + c2 + c3) * a;
            return b;
        }

        public static Quat Mat2Quat(Matrix<double> A)
        {
            if (!(A.IsSquare()) || !(A.Length == 3))
            {
                throw new ArgumentException("Matrix must be square 3x3 matrix.");
            }
            double eta = 0.5 * Math.Sqrt(1.0 + Matrix<double>.Trace(A));
            double eps1 = (A[2, 3] - A[3, 2]) / (4.0 * eta);
            double eps2 = (A[3, 1] - A[1, 3]) / (4.0 * eta);
            double eps3 = (A[1, 2] - A[2, 1]) / (4.0 * eta);
            return new Quat(eta, eps1, eps2, eps3);
        }
        
        /// <summary>
        /// Spherical linear interpolation between two quaternions q0 and q1 given double t from 0 to 1, where t = 0.0 => q0, and t = 1.0 => q1
        /// </summary>
        /// <param name="t"></param>
        /// <param name="q0"></param>
        /// <param name="q1"></param>
        /// <returns name="qInterp"></returns>
        public static Quat Slerp(double t, Quat q0, Quat q1)
        {
            const double THRESHOLD = 0.9995;
            double dot = q0._eta * q1._eta + Vector.Dot(q0._eps, q1._eps);
            if (dot < 0.0)
            {
                q1._eta = -1.0 * q1._eta;
                q1._eps = -1.0 * q1._eps;
                dot = -1.0 * dot;
            }
            if (dot > THRESHOLD)
            {
                // Linear interpolate quaternion for small interpolations 
                Quat qLinterp = new Quat(q0._eta + t * (q1._eta - q0._eta), q0._eps + t * (q1._eps - q0._eps));
                return qLinterp;
            }
            double theta0 = System.Math.Acos(dot);
            double theta = theta0 * t;
            double sTheta = System.Math.Sin(theta);
            double sTheta0 = System.Math.Sin(theta0);
            double cTheta = System.Math.Cos(theta);
            double s0 = cTheta - dot * sTheta / sTheta0;
            double s1 = sTheta / sTheta0;
            Quat qInterp = new Quat(s0 * q0._eta + s1 * q1._eta, s0 * q0._eps + s1 * q1._eps);
            return qInterp;

        }

        public static Quat operator *(Quat q, Quat p)
        {
            double a = q._eta;
            double b = q._eps[1];
            double c = q._eps[2];
            double d = q._eps[3];
            double e = p._eta;
            double f = p._eps[1];
            double g = p._eps[2];
            double h = p._eps[3];
            double eta = e * a - b * f - c * g - d * h;
            double eps1 = a * f + b * e + c * h - d * g;
            double eps2 = a * g + c * e - b * h + d * f;
            double eps3 = a * h + d * e + b * g - c * f;
            return new Quat(eta, eps1, eps2, eps3);
        }

        public override string ToString()
        {
            return "[" + _eta + "," + _eps[1] + ", " + _eps[2] + ",  " + _eps[3] +"]";
        }
    }
}

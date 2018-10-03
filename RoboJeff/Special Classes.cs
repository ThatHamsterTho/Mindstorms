using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;

namespace RoboJeff_Special_classes
{
    class MPoint {

        public double x, y;

        public MPoint(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }


    class Line {

        public MPoint P1, P2;
        public double RC;
        public double[,] range = new double[2, 2];

        public Line(MPoint P1, MPoint P2) {
            this.P1 = P1;
            this.P2 = P2;
            this.RC = (P2.y - P1.y) / (P2.x - P1.x);
            this.range = new double[2, 2] {
                { P1.x, P2.x},
                { P1.y, P2.y}
            };
        }
    }

    class Function{

        public double rc, b;
        public Function(double rc, double b){
            this.rc = rc;
            this.b = b;
        }
        /// <summary>
        /// returns y value at given x position
        /// </summary>
        /// <param name="x"> x value</param>
        /// <returns> y value</returns>
        public double at_x(double x){
            return rc * x + b;
        }
        /// <summary>
        /// return x value at given y position
        /// </summary>
        /// <param name="y">y value</param>
        /// <returns>x value</returns>
        public double at_y(double y){
            return (y / rc) - b;
        }
    }

    class Rect {

        public MPoint P1, P2;
        public double width, height;
        public double[,] range = new double[2, 2];

        public Rect(MPoint P1, MPoint P2) {
            this.P1 = P1;
            this.P2 = P2;
            this.width = P2.x - P1.x;
            this.height = P2.y - P1.y;
            this.range = new double[2, 2] {
                { P1.x, P2.x },
                { P1.y, P2.y}
            };
        }

        public bool intersect(Line l) {
            bool in_x_range = false, in_y_range = false;
            if(l.P1.x <= this.P2.x && this.P1.x <= l.P2.x){
                in_x_range = true;
                    }
            if(l.P1.y <= this.P2.y && this.P1.y <= l.P2.y){
                in_y_range = true;
                    }
            if(in_x_range && in_y_range){
                return true;
            }
            else {
                return false;
            }

            // source -> https://stackoverflow.com/questions/13513932/algorithm-to-detect-overlapping-periods
        }
    }
}

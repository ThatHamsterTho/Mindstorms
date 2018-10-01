using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;

namespace RoboJeff_Special_classes
{
    class MPoint {

        public int x, y;

        public MPoint(int x, int y)
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
                

        }
    }
}

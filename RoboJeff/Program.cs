using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RoboJeff
{

    public class Challenge
    {
        public int[] hit_box, pos;
        public int angle;
        public string name;

        public Challenge(int x, int y, int angle, int[] hitbox, string name)
        {
            this.hit_box = hitbox;
            this.pos[0] = x;
            this.pos[1] = y;
            this.angle = angle;
            this.name = name;
        }
    }

    class Robot
    {
        double angle = 0;                   // in degrees
        double[] rel_triangle = new double[3];
        int[] pos = { 0, 0 };
        int[] hit_box = { 0, 0, 20, 15 };   // change to size of robot
        double err_marge = 0.5;
        double[] wheel_sizes = { 4.3, 5.6, 6.9 };
        double min_rot = 0.1;
        double axle = 8;                    // change to size of axle
        int wheel = 0;

        public void rotate(double angle)
        {
            double perc = angle / 360;
            double rotations = perc * (this.axle / 2 * Math.PI) / this.wheel_sizes[this.wheel];
            this.angle = angle;
        }

        public double[] rotate_to_chall(Challenge challenge)
        {
            int x = challenge.pos[0];
            int y = challenge.pos[1];
            double dx = x - this.pos[0];
            double dy = y - this.pos[1];
            double rc = dy / dx;
            double angle = Math.Abs(Math.Atan(rc));
            this.rotate(angle * (180 / Math.PI));
            double[] result = { dx, dy };
            return result;
        }

        public void goto_chall(Challenge challenge)
        {
            double[] result = this.rotate_to_chall(challenge);
            double dx = result[0];
            double dy = result[1];

            double rel_dist = Math.Sqrt(Math.Exp(dx) + Math.Exp(dy));
            double[] rel_triangle = { rel_dist, dx, dy };
            this.rel_triangle = rel_triangle;
            double rotations = rel_dist / this.wheel_sizes[this.wheel]
        }

        public void check_path(Challenge challenge)
        {

        }

        public void rotate_at_end(Challenge challenge)
        {
            this.rotate(challenge.angle - this.angle);
        }
    }

    class Motor
    {
        Robot robot;

        public Motor(Robot robot)
        {
            this.robot = robot;
        }
    }

    class Program
    {

        static void Main(string[] args)
        {
            double[] board_size = { 238.2, 114.3 };
        }
    }
}

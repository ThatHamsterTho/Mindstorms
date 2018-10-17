using System;
using System.Threading;

using System.Reflection;
using System.Resources;

using MonoBrickFirmware.Sound;

using MonoBrickFirmware.Display.Dialogs;
using MonoBrickFirmware.Sensors;
using MonoBrickFirmware.Display;
using MonoBrickFirmware.UserInput;
using MonoBrickFirmware.Movement;
using RoboJeff_Special_classes;

namespace RoboJeff
{
    /* 
     * Using Classes for readability
     * Classes aren't actually necessarry but it helps with reading the code as you know that a function should be doing something related to the class name
     * 
     * 
     * 
     */

    // variables for text
    public class vars
    {
        public static Font f = Font.MediumFont;                             // font size
        public static Point offset = new Point(0, 12);                      // offset point
        public static Point p = new Point(10, Lcd.Height - 75);             // point?
        public static Point boxSize = new Point(100, 24);                   // boxsize
        public static Rectangle box = new Rectangle(p, p + boxSize);        // rectangle of the box

        /// source -> https://github.com/Larsjep/monoev3/blob/release/LcdExample/Program.cs

        public static void print(string text)
        {
            Lcd.Clear();
            Lcd.WriteTextBox(f, box, text, true);
            Lcd.Update();
        }
    }


    // Sensor class -> contains all the objects for the sensors and functions for the sensor
    public class V_Sensor // virtual sensor
    {
        public EV3UltrasonicSensor US = new EV3UltrasonicSensor(SensorPort.In1);              // Ultrasonic sensor object     used
        public EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.In4, GyroMode.Angle);        // Gyro sensor object           used
                                                                                              //public EV3TouchSensor touch = new EV3TouchSensor(SensorPort.In3);                   // touch sensor object          not used
                                                                                              //public EV3ColorSensor color = new EV3ColorSensor(SensorPort.In1);                   // color sensor object          not used
        public EV3GyroSensor reset_gyro = new EV3GyroSensor(SensorPort.In4, GyroMode.Angle);  // resettable gyro sensor       used

        /* colour in string value
        public string read_color()
        {
            return color.ReadAsString();
        }
        */

        // value in mm?;
        public int read_US()
        {
            return US.Read();
        }

        // returns value from 360 to -360, negative is turning from right to left, positive is from left to right
        public int read_gyro()
        {
            return gyro.Read();
        }

        public int read_gyro_r()
        {
            return reset_gyro.Read();
        }

        public void reset_gyro_r()
        {
            reset_gyro.Reset();
        }

        /* return booleon value of true when pressed and false when not pressed;
        public bool read_touch()
        {
            if (touch.Read() == 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        */
    }


    public class V_Motor // Virtual Motor
    {
        // each movement function has its own standard TCPR ( tacho_count_per_rotation ) value due to the use of different functions.

        public V_Sensor vsensor = new V_Sensor();
        public Motor motorR = new Motor(MotorPort.OutA);                                // left motor ( faced from input input )
        public Motor motorL = new Motor(MotorPort.OutD);                                // right motor ( faced from input buttons )
        public Motor motorArm = new Motor(MotorPort.OutB);                              // precise motor
        public Vehicle Robot_Vehicle = new Vehicle(MotorPort.OutA, MotorPort.OutD);     // full vehicle control object

        const sbyte speed = 127;                                                         // speed the motors rotate around their axis
        const sbyte turn_speed = 10;                                                    // speed the robot turns around
        const uint tcpr = 360;


        // forward function using vehicle
        public void forward(double rotations, Robot robot)
        {
            Robot_Vehicle.ReverseLeft = false;
            Robot_Vehicle.ReverseRight = false;

            // (waitHandle) wait_event <- Robot_Vehicle.Forward(speed, tacho_count, brake) | brake -> true
            motorL.ResetTacho();
            motorR.ResetTacho();

            double tacho_count = tcpr * rotations;

            WaitHandle wait_event = Robot_Vehicle.Forward(speed, ((uint)tacho_count), true);
            wait_event.WaitOne();

            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        public void backward(double rotations, Robot robot)
        {
            Robot_Vehicle.ReverseLeft = false;
            Robot_Vehicle.ReverseRight = false;

            motorL.ResetTacho();
            motorR.ResetTacho();

            double tacho_count = tcpr * rotations;

            WaitHandle wait_event = Robot_Vehicle.Backward(speed, ((uint)tacho_count), true);
            wait_event.WaitOne();

            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        // do not input 360 or 0 degrees due to loop back error
        public void Rotate(double degrees, Robot robot, bool dir = true)
        {
            if(degrees < 0 && !dir)
            {
                degrees = 360 + degrees;
            }

            ButtonEvents buts = new ButtonEvents();
            bool done = false;

            vsensor.reset_gyro_r();

            // spin left
            if (dir)
            {
                // left wheel forwards right wheel backwards
                motorL.SetSpeed(turn_speed);
                motorR.SetSpeed((sbyte)-turn_speed);

            }
            // spin right
            else
            {
                // left wheel forwards right wheel backwards
                motorL.SetSpeed((sbyte)-turn_speed);
                motorR.SetSpeed(turn_speed);

            }
            buts.EscapePressed += () => {
                done = true;
            };

            double new_turned = 0; // needed because of when vsensor is called after the while loop it will add more degrees than the currently turned amount ( seems to be a bug in the libraries )
            if (degrees < 0)
            {
                do
                {
                    vars.print($"diff to:  {vsensor.read_gyro_r()}");
                    new_turned = vsensor.read_gyro_r();
                }
                while (vsensor.read_gyro_r() >= degrees && !done);
            }
            else
            {
                do
                {
                    vars.print($"diff to: {vsensor.read_gyro_r()}");
                    new_turned = vsensor.read_gyro_r();
                }
                while (vsensor.read_gyro_r() <= degrees && !done);
            }

            motorL.Brake();
            motorR.Brake();

            vars.print($"{robot.angle} + {new_turned.ToString()} = {robot.angle + new_turned}");
            Thread.Sleep(3000);

            double curr_turned = robot.angle;
            double new_angle = curr_turned + new_turned;

            if(new_angle > 180)
            {
                new_angle = new_angle - 360;
            }
            if (new_angle < -180)
            {
                new_angle = new_angle + 360;
            }

            robot.angle = robot.angle + new_turned;
        }

        public void MoveArm(double degrees, int speed = 100, bool down = false, int time = 0)
        {
            if (degrees < 0)
            {
                speed = speed * -1;
                degrees = degrees * -1;
            }
            motorArm.ResetTacho();

            degrees = degrees * 2.5;

            WaitHandle WaitHandle_up = motorArm.PowerProfile((sbyte)speed, 0, (uint)degrees, 0, true);
            WaitHandle_up.WaitOne();
            if (down)
            {
                speed = speed * -1;
                Thread.Sleep(time);
                WaitHandle WaitHandle_down = motorArm.PowerProfile((sbyte)speed, 0, (uint)(degrees), 0, true);
                WaitHandle_down.WaitOne();
            }

            motorArm.Off();
        }
    }

    // virtual challenge ( contains information such as the position of the challenge and the size of it )
    public class Challenge
    {
        // hitbox is full area of challenge
        // pos is the position the robot should stand relative to the challenge
        // angle is the angle the robot should stand to complete the challenge
        // name is the name of the challenge ( for reference )
        public double[] hit_box = new double[4], pos = new double[2];
        public double angle;
        public string name;

        // constructor
        public Challenge(double x, double y, double angle, double[] hitbox, string name)
        {
            this.hit_box = hitbox;
            this.pos[0] = x;
            this.pos[1] = y;
            this.angle = angle;
            this.name = name;
        }


    }


    // robot class -> this is the virtual version of the robot
    public class Robot
    {
        /*	
		 * triangle referance for gionomotry
		 * 							c
		 * 							. y2
		 * 						  - |
		 * 						-	|
		 * 					  -		|
		 * 					-		|
		 * 			  C	  -			|	B
		 * 				-			|
		 * 			  -				|
		 * 			-				|
		 * 		  -					|
		 * 		x1------------------] x2, y1
		 * 	  a			  A          b
		 */

        public double angle = 0;                                     // the angle the robot stands relative to start point in degrees
        public double axle = 14;                                     // diameter of turning circle ( ball bearing end not accounted for )

        public double[] pos = { 44.5, 25.5 };                        // starting position robot 
        public double[] hit_box = { 0, 0, 31, 14 };                  // the hitbox of the robot

        public double[] wheel_sizes = { 4.3, 5.6, 6.9 };             // the diameters of the wheel sizes
        public int wheel = 2;                                        // current wheel size being used.

        public double[] scale_triangle = new double[3];              // the triange of the path the robot is taking C being the path A being the X-size and B being the Y-size

        public V_Motor vmotor = new V_Motor();
        public V_Sensor vsensor = new V_Sensor();

        // challenges
        public Challenge[] Challenges = new Challenge[] {
            new Challenge(14, 95, 0, new double[] { 3, 102, 91, 108 }, "M1"),
            new Challenge(94, 75, 0, new double[] { 57.5, 65, 70.5, 95.5 }, "M4"),
            new Challenge(71.5, 45, -90, new double[] { 65, 58.5, 77.5, 64 }, "M5"),
            new Challenge(172, 101.5, -180, new double[] {142, 95, 165, 107 }, "M8"),
            new Challenge(129, 101.5, 0, new double[] { 142, 95, 165, 107 }, "M9"),
            new Challenge(153, 77.5, 0, new double[] { 153, 77.5, 166, 90 }, "M10"),
            new Challenge(112, 30, 0, new double[] { 112, 30, 138, 56 }, "M6"),
            
        };
        public Challenge[] Nodes = new Challenge[]
        {
            new Challenge(44.5, 25.5, 0, new double[] { 0, 0, 0, 0 }, "basis"),
            new Challenge(92, 44.5, 0, new double[] {0,0,0,0}, "N1_toM4"),
            new Challenge(54, 80, 0, new double[] {0,0,0,0}, "N2_fromM4"),
            new Challenge(129, 25, 0, new double[] {0,0,0,0}, "N3_toN4"),
            new Challenge(162, 42, 0, new double[] {0,0,0,0}, "N4_toN5"),
            new Challenge(159, 61, 0, new double[] {0,0,0,0}, "N4_toDUWCHALL")
        };

        // rotates

        public void rotate(double angle, Robot robot)
        {
            /* theoretical way of turning
			 * 
			 * double perc = angle / 360;														<- the scale factor to turn the robot
			 * double rotations = perc * (this.axle * Math.PI) / this.wheel_sizes[this.wheel];  <-  this.axle * Math.PI = circumference, circ * perc = amount needed to rotate circ, 
			 * 																						devide that by the current wheelsize to get amount of rotations needed to travel that distance
			 * this.angle = angle;																<- setting the angle to new angle
			 */

            // any value above 180 can be a smaller rotation when translated to a negative rotation
            if (angle > 0)
            {
                if (angle > 180)
                {
                    angle = angle - 360;
                }
            }
            else
            {
                if(angle < -180)
                {
                    angle = angle + 360;
                }
            }


            // if the angle is less than 0 rotate right to left
            // if the angle is bigger than 0 rotate left to right
            if (angle > 1 || angle < -1)
            {
                if (angle < 0)
                {
                    vmotor.Rotate(angle, robot, true);
                }
                else
                {
                    vmotor.Rotate(angle, robot, false);
                }
            }
        }

        // rotates to specified challenge

        public double[] rotate_to_chall(Challenge challenge, Robot robot, bool reverse_rotate)
        {
            double x = challenge.pos[0];
            double y = challenge.pos[1];
            double dx = x - pos[0];                                                     // getting A side of triangle
            double dy = y - pos[1];                                                     // getting B side of triangle
            double rc = dy / dx;                                                        // rc needed to get rotation needed to face destination ( atan(rc) = degrees relative to x-axis
            double angle;
            if (rc > 0) { angle = (Math.Atan(rc) * (180 / Math.PI)) * -1; }                  // getting angle needed to face destination
            else { angle = 180 + (Math.Atan(rc) * (180 / Math.PI)) * -1; }
            vars.print($"rotate to: {angle}");
            Thread.Sleep(1000);
            vars.print($"current: {robot.angle}");
            Thread.Sleep(1000);
            if (Math.Abs(angle - robot.angle) < 2)
            {
                angle = 180;
            }
            else
            {
                angle = (angle - robot.angle);                                          // gets the difference between new angle and current angle and sets that as rotation angle
                if (reverse_rotate)
                {
                    angle = -180 + angle;
                }
            }
            vars.print($"diff to: {angle}");
            Thread.Sleep(1000);
            this.rotate(angle, robot);                                                  // rotates to destination
            double[] result = { dx, dy };                                               // returns A and B side of triangle

            return result;
        }

        // go to specified challenge
        public void goto_chall(Challenge challenge, Robot robot, AutoResetEvent wait, bool rotate_at_end = true, bool reverse_rotate = false)
        {
            double[] result = this.rotate_to_chall(challenge, robot, reverse_rotate);                   // rotates to challenge
            double dx = result[0];                                                      // gets A side of triangle
            double dy = result[1];                                                      // gets B side of triangle

            double rel_dist = Math.Sqrt((dx * dx) + (dy * dy));                         // gets C side of triangle sqrt( C^2 = A^2 + B^2 ) = C
            double[] rel_triangle = { rel_dist, dx, dy };                               // creates the rel_triangle
            scale_triangle = rel_triangle;                                              // sets the scale_triangle ( to rel_triangle )
            double rotations = rel_dist / (wheel_sizes[wheel] * Math.PI);               // calculates the rotations needed to go to challenge
            vmotor.forward(rotations, robot);                                           // move forward for rotations
            pos = challenge.pos;
            if (rotate_at_end) { this.rotate_at_end(challenge, robot); }
            wait.Set();
        }

        // TODO: add smart movement around objects


        // rotates to the challenge when arrived at the designated challenge stop point
        public void rotate_at_end(Challenge challenge, Robot robot)
        {
            angle = (challenge.angle - this.angle);                                // gets the difference between challenge angle and current angle and sets that as rotation angle
            this.rotate(angle, robot);
        }
    }


    /// MAIN
    class MainClass
    {
        public static void Main(string[] args)
        {
            // TODO: check x, y position and angle the robot should stand to challenge

            Robot robot = new Robot();

            V_Motor vmotor = new V_Motor();
            V_Sensor vsensor = new V_Sensor();

            AutoResetEvent wait = new AutoResetEvent(false);
            ButtonEvents buts = new ButtonEvents();

            buts.EnterPressed += () => {
                wait.Set();
            };
            
            vars.print("going to node 1");
            robot.goto_chall(robot.Nodes[1], robot, wait, false);
            vars.print("node 1 reached!");
            wait.WaitOne(); wait.Reset();
            vars.print("going to challenge");
            robot.goto_chall(robot.Challenges[1], robot, wait, false);
            vars.print("challenge reached!");
            wait.WaitOne(); wait.Reset();
            vars.print("going to node 2");
            robot.goto_chall(robot.Nodes[2], robot, wait, false);
            wait.WaitOne(); wait.Reset();
            vars.print("node 2 reached!");
            vmotor.forward(3, robot);
            vars.print("going to basis");
            robot.goto_chall(robot.Nodes[0], robot, wait, false, true);
            wait.WaitOne();
            vars.print("basis reached!");
        }
    }


    /// programming notes
    /// use *waithandle_object*.waitOne(0) to check if the waithandle has been set or not
    /// When using the motors's tacho count they seem to sometimes work with these settings but can break for an unknown reason
    /// suspecting that the tacho count is altered before the program which brings the robot off balance
    /// reason of suspection is that after reusing a few values ( doesn't matter what values ) they seem to work properly again
    /// if robot is acting up call the movement functions in the Robot.vmotor to readjust the values this might take a few reuploads of the program
    /// 
    /// Dont use the rotate class with 0 or 360 degrees because those will cause a loop back and make the robot spin forever


}
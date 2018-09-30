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

namespace RoboJeffV2
{
    // variables for text
    public class vars
    {
        public static Font f = Font.MediumFont;                         // font size
        public static Point offset = new Point(0, 12);                 // offset point
        public static Point p = new Point(10, Lcd.Height - 75); // point?
        public static Point boxSize = new Point(100, 24);       // boxsize
        public static Rectangle box = new Rectangle(p, p + boxSize);   // rectangle of the box
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
        public EV3UltrasonicSensor US = new EV3UltrasonicSensor(SensorPort.In1);            // Ultrasonic sensor object
        public EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.In2, GyroMode.Angle);      // Gyro sensor object
        public EV3TouchSensor touch = new EV3TouchSensor(SensorPort.In3);                   // touch sensor object
        public EV3ColorSensor color = new EV3ColorSensor(SensorPort.In4);                   // color sensor object
        public EV3GyroSensor reset_gyro = new EV3GyroSensor(SensorPort.In2, GyroMode.Angle); // resettable gyro sensor

        // colour in string value
        public string read_color()
        {
            return color.ReadAsString();
        }

        // value in mm?;
        public int read_US()
        {
            return US.Read();
        }

        // returns value from 180 to -180, negative is turning from left to right, positive is from right to left
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

        // return booleon value of true when pressed and false when not pressed;
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

        public double angle = 0;                           // the angle the robot stands relative to start point in degrees
        public double[] scale_triangle = new double[3];    // the triange of the path the robot is taking C being the path A being the X-size and B being the Y-size
        public int[] pos = { 19, 7 };                       // the position of the robot relative to the robot 
        public int[] hit_box = { 0, 0, 31, 14 };           // the hitbox of the robot

        public double err_marge = 0.5;                     // the marge of error the robot would generally take when arriving to destination
        public double[] wheel_sizes = { 4.3, 5.6, 6.9 };   // the diameters of the wheel sizes
        public double min_rot = 0.1;                       // minimum of wheel rotations done by robot
        public double axle = 14;                            // diameter of turning circle ( ball bearing end not accounted for )
                                                            /// CHANGE THIS
        public int wheel = 1;                              // current wheel size being used.


        // rotates

        public void rotate(double angle)
        {
            /* theoretical way of turning
			 * 
			 * double perc = angle / 360;														<- the scale factor to turn the robot
			 * double rotations = perc * (this.axle * Math.PI) / this.wheel_sizes[this.wheel];  <-  this.axle * Math.PI = circumference, circ * perc = amount needed to rotate circ, 
			 * 																						devide that by the current wheelsize to get amount of rotations needed to travel that distance
			 * this.angle = angle;																<- setting the angle to new angle
			 */

            // change angle that is less than 0 to 180 - angle
            if (angle < 0)
            {
                this.turn(180 + angle);
            }
            else
            {
                this.turn(angle);
            }
        }

        // rotates to specified challenge

        public double[] rotate_to_chall(Challenge challenge)
        {
            double x = challenge.pos[0];
            double y = challenge.pos[1];
            double dx = x - this.pos[0];                // getting A side of triangle
            double dy = y - this.pos[1];                // getting B side of triangle
            double rc = dy / dx;                        // rc needed to get rotation needed to face destination ( atan(rc) = degrees relative to x-axis
            double angle = Math.Atan(rc);               // getting angle needed to face destination
            this.rotate(angle * (180 / Math.PI));       // rotates to destination
            double[] result = { dx, dy };               // returns A and B side of triangle
            return result;
        }

        // go to specified challenge
        public void goto_chall(Challenge challenge)
        {
            double[] result = this.rotate_to_chall(challenge);          // rotates to challenge
            double dx = result[0];                                      // gets A side of triangle
            double dy = result[1];                                      // gets B side of triangle

            double rel_dist = Math.Sqrt((dx * dx) + (dy * dy));   // gets C side of triangle sqrt( C^2 = A^2 + B^2 ) = C
            double[] rel_triangle = { rel_dist, dx, dy };               // creates the rel_triangle
            this.scale_triangle = rel_triangle;                         // sets the scale_triangle ( to rel_triangle )
            double rotations = rel_dist / (this.wheel_sizes[this.wheel] * Math.PI); // calculates the rotations needed to go to challenge
            this.forward(rotations);                                    // move forward for rotations
        }

        public void check_path(Challenge challenge)
        {
            /// ADD THIS -> needed to check if there is anything in the path of the robot
            // TODO: add check_path function in robot class
        }

        // rotates to the challenge when arrived at the designated challenge stop point
        public void rotate_at_end(Challenge challenge)
        {
            this.rotate(challenge.angle - this.angle);
        }


        /// MOTOR FUNCTIONS START HERE
        // each movement function has its own standard TCPR ( tacho_count_per_rotation ) value due to the use of different functions.

        public V_Sensor vsensor = new V_Sensor();
        public Motor motorR = new Motor(MotorPort.OutA);                        // left motor ( faced from input input )
        public Motor motorL = new Motor(MotorPort.OutD);                            // right motor ( faced from input buttons )
        public Motor motorArm = new Motor(MotorPort.OutB);                          // precise motor
        public Vehicle Robot_Vehicle = new Vehicle(MotorPort.OutA, MotorPort.OutD);     // full vehicle control object

        sbyte speed = 50;                                                   // speed the motors rotate around their axis
        sbyte turn_speed = 10;                                              // speed the robot turns around

        // forward function using vehicle
        public void forward(double rotations)
        {
            const uint tcpr = 360;
            Robot_Vehicle.ReverseLeft = false;
            Robot_Vehicle.ReverseRight = false;

            double tacho_count = tcpr * rotations;
            int x = 0;
            WaitHandle wait_event = Robot_Vehicle.Forward(speed, ((uint)tacho_count), true);
            while (!wait_event.WaitOne(0))
            {   // the 0 indicates that the following code is not blocked by Waiting for the event to be set
                x++;
            }

            vars.print(x.ToString());

            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        // do not input 360 or 0 degrees due to loop back error
        public void turn(double degrees, bool dir = true)
        {
            const uint tcpr = 350;
            const int error_marge = 0;
            degrees = Math.Abs(degrees) - error_marge;

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
            double curr_turned = 0; // needed because of when vsensor is called after the while loop it will add more degrees than the currently turned amount ( seems to be a bug in the libraries )
            while (Math.Abs(vsensor.read_gyro_r()) <= degrees && !done)
            {
                buts.EscapePressed += () => {
                    done = true;
                };
                vars.print(vsensor.read_gyro().ToString());
                curr_turned = vsensor.read_gyro_r();
            }

            motorL.Brake();
            motorR.Brake();

            this.angle = curr_turned;
        }
    }


    /// MAIN
    class MainClass
    {
        public static void Main(string[] args)
        {
            /// broke program?
            // NOTE: Problem was a non-assigned length double array
            // TODO: check x, y position and angle the robot should stand to challenge

            Challenge chal_1 = new Challenge(3, 102, 0, new double[] { 3, 102, 91, 108 }, "M1");
            Challenge chal_2 = new Challenge(57.5, 65, 0, new double[] { 57.5, 65, 70.5, 95.5 }, "M4");
            Challenge chal_3 = new Challenge(65, 58.5, 0, new double[] { 65, 58.5, 77.5, 64 }, "M5");
            Challenge chal_4 = new Challenge(142, 95, 0, new double[] { 142, 95, 165, 107 }, "M9");
            Challenge chal_5 = new Challenge(153, 77.5, 0, new double[] { 153, 77.5, 166, 90 }, "M10");
            Challenge chal_6 = new Challenge(112, 30, 0, new double[] { 112, 30, 138, 56 }, "M6");

            Robot robot = new Robot();

            robot.forward(5);
        }
    }






}
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

        const sbyte speed = 50;                                                         // speed the motors rotate around their axis
        const sbyte turn_speed = 10;                                                    // speed the robot turns around
        const uint tcpr = 360;


        // forward function using vehicle
        public void forward(double rotations)
        {
            Robot_Vehicle.ReverseLeft = false;
            Robot_Vehicle.ReverseRight = false;

            motorL.ResetTacho();
            motorR.ResetTacho();

            double tacho_count = tcpr * rotations;

            WaitHandle wait_event = Robot_Vehicle.Forward(speed, ((uint)tacho_count), true);
            wait_event.WaitOne();

            int tmr = motorR.GetTachoCount();
            int tml = motorL.GetTachoCount();
            int tm_average = (int)(tmr + tml) / 2;
            double dist_travelled = (tm_average / tcpr) * Robot.wheel_sizes[Robot.wheel];
            double fac = dist_travelled / Robot.scale_triangle[0];
            Robot.pos[0] = fac * Robot.scale_triangle[1];
            Robot.pos[1] = fac * Robot.scale_triangle[2];

            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        public void backward(double rotations){
            Robot_Vehicle.ReverseLeft = false;
            Robot_Vehicle.ReverseRight = false;

            motorL.ResetTacho();
            motorR.ResetTacho();

            double tacho_count = tcpr * rotations;

            WaitHandle wait_event = Robot_Vehicle.Backward(speed, ((uint)tacho_count), true);
            wait_event.WaitOne();

            int tmr = motorR.GetTachoCount();
            int tml = motorL.GetTachoCount();
            int tm_average = (int)(tmr + tml) / 2;
            double dist_travelled = (tm_average / tcpr) * Robot.wheel_sizes[Robot.wheel];
            double fac = dist_travelled / Robot.scale_triangle[0];
            Robot.pos[0] = fac * Robot.scale_triangle[1];
            Robot.pos[1] = fac * Robot.scale_triangle[2];

            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        // do not input 360 or 0 degrees due to loop back error
        public void Rotate(double degrees, bool dir = true)
        {
            const int error_marge = 0;
            degrees = Math.Abs(degrees) - error_marge;

            ButtonEvents buts = new ButtonEvents();
            bool done = false;

            vsensor.reset_gyro_r();
            vars.print(degrees.ToString());
            Thread.Sleep(2000);

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
                vars.print(vsensor.read_gyro_r().ToString());
                curr_turned = vsensor.read_gyro_r();
            }

            motorL.Brake();
            motorR.Brake();

            Robot.angle = curr_turned;
        }

        public void MoveArm(double degrees, int speed = 100, bool down = false, int time = 0)
        {
            if (degrees < 0)
            {
                speed = speed * -1;
                degrees = degrees * -1;
            }
            motorArm.ResetTacho();

            WaitHandle WaitHandle_up = motorArm.PowerProfile((sbyte)speed, 0, (uint)degrees, 0, true);
            WaitHandle_up.WaitOne();
            if(down){
                Thread.Sleep(time);
                WaitHandle WaitHandle_down = motorArm.PowerProfile((sbyte)-speed, 0, (uint)-degrees, 0, true);
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

        static public double angle = 0;                                     // the angle the robot stands relative to start point in degrees
        static public double axle = 14;                                     // diameter of turning circle ( ball bearing end not accounted for )

        static public double[] pos = { 44.5, 25.5 }; // 24, 26 };                            // starting position robot 
        static public double[] hit_box = { pos[0], pos[1], 31+pos[0], 14+pos[1] };                  // the hitbox of the robot

        static public double[] wheel_sizes = { 4.3, 5.6, 6.9 };             // the diameters of the wheel sizes
        static public int wheel = 1;                                        // current wheel size being used.

        static public double[] scale_triangle = new double[3];              // the triange of the path the robot is taking C being the path A being the X-size and B being the Y-size

        static public V_Motor vmotor = new V_Motor();

        // challenges
        static public Challenge[] Challenges = new Challenge[] {
            new Challenge(14, 95, 0, new double[] { 3, 102, 91, 108 }, "M1"),
            new Challenge(57.5, 65, 0, new double[] { 57.5, 65, 70.5, 95.5 }, "M4"),
            new Challenge(65, 58.5, 0, new double[] { 65, 58.5, 77.5, 64 }, "M5"),
            new Challenge(142, 95, 0, new double[] { 142, 95, 165, 107 }, "M9"),
            new Challenge(153, 77.5, 0, new double[] { 153, 77.5, 166, 90 }, "M10"),
            new Challenge(112, 30, 0, new double[] { 112, 30, 138, 56 }, "M6"),
            new Challenge(20, 20, 0, new double[] { 0, 0, 0, 0 }, "basis"),
        };
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
                vmotor.Rotate(180 + angle);
            }
            else
            {
                vmotor.Rotate(angle);
            }
        }

        // rotates to specified challenge

        public double[] rotate_to_chall(Challenge challenge)
        {
            double x = challenge.pos[0];
            double y = challenge.pos[1];
            double dx = x - pos[0];                                                     // getting A side of triangle
            double dy = y - pos[1];                                                     // getting B side of triangle
            double rc = dy / dx;                                                        // rc needed to get rotation needed to face destination ( atan(rc) = degrees relative to x-axis
            double angle = Math.Atan(rc);                                               // getting angle needed to face destination
            this.rotate(angle * (180 / Math.PI));                                       // rotates to destination
            double[] result = { dx, dy };                                               // returns A and B side of triangle
            return result;
        }

        // go to specified challenge
        public void goto_chall(Challenge challenge, ManualResetEvent wait)
        {
            double[] result = this.rotate_to_chall(challenge);                          // rotates to challenge
            double dx = result[0];                                                      // gets A side of triangle
            double dy = result[1];                                                      // gets B side of triangle

            double rel_dist = Math.Sqrt((dx * dx) + (dy * dy));                         // gets C side of triangle sqrt( C^2 = A^2 + B^2 ) = C
            double[] rel_triangle = { rel_dist, dx, dy };                               // creates the rel_triangle
            Robot.scale_triangle = rel_triangle;                                        // sets the scale_triangle ( to rel_triangle )
            double rotations = rel_dist / (Robot.wheel_sizes[Robot.wheel] * Math.PI);   // calculates the rotations needed to go to challenge
            vmotor.forward(rotations);                                                  // move forward for rotations
            this.rotate_at_end(challenge);
            wait.Set();
        }

        public Challenge[] check_path(Challenge challenge, out bool safe)
        {
            Challenge[] new_route = new Challenge[2];

            /// ADD THIS -> needed to check if there is anything in the path of the robot
            // TODO: add check_path function in robot class

            /* NOTE:
            *  to check path, you need 2 lines and a rectangular hitbox
            *  the 1st based on the left side of the robot and the 2nd is based on the right side
            *  if either line hits the rectangle a new route is needed
            *  both lines are based on the middle part of the robot, where the x and y coordinates
            *  are known and then half the diameter is removed and added from the starting value (b)
            *  to get the new lines
            */

            /* get the middle route line:
            *  y = rc * x + b <- needing this type of function to get 2nd pos
            *  y and x are known in the pos array: pos[0], pos[1]
            *  rc is known by the inverse tan of the current angle: Math.Atan(angle)
            *  and b is known by: (y / rc) - x = b
            */
            
            // creates point 2 needed to make the line
            // the x is the distance towards the destination
            double dx = challenge.pos[0] - pos[0];                                      // gets A side of triangle
            double dy = challenge.pos[1] - pos[1];                                      // gets B side of triangle
            double rel_dist = Math.Sqrt((dx * dx) + (dy * dy));                         // gets C side of triangle sqrt( C^2 = A^2 + B^2 ) = C
            
            double angle = Math.Atan(dy/dx);
            double b = (pos[1] / Math.Tan(angle)) - pos[0];
            double y = Math.Atan(angle) * rel_dist + b;                                 // the middle route line
            double b_diff = 1 / Math.Cos(angle) * (axle / 2);                           // this is the b difference modifier, needed to keep the lines at a constant distance from each other
            // point 1 is the current position of the robot
            // point 2 is the destination position of the robot (rel_dist, y)             
            MPoint originL = new MPoint(pos[0], pos[1] + b_diff);
            MPoint originR = new MPoint(pos[0], pos[1] - b_diff);
            MPoint destL = new MPoint(rel_dist, y + b_diff);
            MPoint destR = new MPoint(rel_dist, y - b_diff);
            Line L_routeL = new Line(originL, destL);
            Line L_routeR = new Line(originL, destL);
            Rect Chal_rect = new Rect(new MPoint(challenge.hit_box[0], challenge.hit_box[1]),
                                      new MPoint(challenge.hit_box[2], challenge.hit_box[3]));

            if(Chal_rect.intersect(L_routeL) || Chal_rect.intersect(L_routeR)){
                Function F_routeL = new Function(Math.Tan(angle), b + b_diff);
                Function F_routeR = new Function(Math.Tan(angle), b - b_diff);
                
                int scenario;
                if(Chal_rect.intersect(L_routeL)){
                    scenario = 1;
                }
                else {
                    scenario = 2;
                }

                switch (scenario){
                    case 0:
                        // both routes intersect.
                        // check the difference of length to both sides, if either side is less go that side
                        // --------*------*---- -> go more right and check path
                        // ---*-----*---------- -> go more left and check path ( check the difference at the end of the rectangle)
                        // possible state, not needed

                        break;
                    case 1:
                        // left route intersects
                        // go more right check new route
                        // get the challenge hitbox[3,1] for the down right corner, and add 15 to both values as the new destination
                        new_route[0] = new Challenge(challenge.hit_box[3] + 15, challenge.hit_box[1] + 15, 0, new double[] {0,0,0,0}, "new_route_1");
                        new_route[1] = challenge;
                        break;
                    
                    case 2:
                        // right route intersects
                        // go more left check new route
                        // get the challenge hitbox[0,4] for the upper left corner, and add 15 to both values as the new destination
                        new_route[0] = new Challenge(challenge.hit_box[0] + 15, challenge.hit_box[4] + 15, 0, new double[] {0,0,0,0}, "new_route_1");
                        new_route[1] = challenge;
                        break;
                }
                safe = false;
                return new_route;
            }
            else{
                safe = true;
                new_route[0] = challenge;
                return new_route;
            }
        }

        public bool create_path(Challenge challenge, ManualResetEvent wait, out Challenge[] new_route){
            bool safe;
            bool drive = true;
            Challenge[] route = check_path(challenge, out safe);;
            foreach(Challenge Chall in Challenges){
                if(Chall != challenge){
                    route = check_path(Chall, out safe);
                    if(!safe){
                        drive = false;
                        break;
                    }
                }
            }

            if(drive){
                goto_chall(challenge, wait);
                new_route = route;
                return true;
            }

            else {
                new_route = route;
                return false;
            }
        }
        // rotates to the challenge when arrived at the designated challenge stop point
        public void rotate_at_end(Challenge challenge)
        {
            // TODO: make this method work
            //this.rotate(challenge.angle - angle);
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


            Robot robot = new Robot();
            
            V_Motor vmotor = new V_Motor();
            
            ManualResetEvent wait = new ManualResetEvent(false);

            Challenge[] new_path;
            if(!robot.create_path(Robot.Challenges[0], wait, out new_path)){
                for(int i = 0; i < new_path.Length; i++){
                    robot.goto_chall(new_path[i], wait);
                    wait.WaitOne();
                    wait.Reset();
                }
            }


            // beep at end of challenge
            Speaker spk = new Speaker(50);
            spk.Beep();
            Thread.Sleep(100);
            spk.Beep();


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
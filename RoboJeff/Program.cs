using MonoBrickFirmware.Display;
using MonoBrickFirmware.Movement;
using MonoBrickFirmware.Sensors;
using MonoBrickFirmware.UserInput;
using System;
using System.Threading;

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

        // debug print function
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
        //public EV3UltrasonicSensor US = new EV3UltrasonicSensor(SensorPort.In1);              // Ultrasonic sensor object   not used
        public EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.In4, GyroMode.Angle);        // Gyro sensor object           used
        //public EV3TouchSensor touch = new EV3TouchSensor(SensorPort.In3);                   // touch sensor object          not used
        //public EV3ColorSensor color = new EV3ColorSensor(SensorPort.In1);                   // color sensor object          not used
        public EV3GyroSensor reset_gyro = new EV3GyroSensor(SensorPort.In4, GyroMode.Angle);  // resetable gyro sensor        used
        /// the resetable gyro is used for the rotate function, it resets every time the robot needs to turn an amount of degrees.
        
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

        // function to read the resetable gyro
        public int read_gyro_r()
        {
            return reset_gyro.Read();
        }

        // function to reset the resetable gyro
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
        const sbyte turn_speed = 10;                                                     // speed the robot turns around
        const uint tcpr = 360;                                                           // Tacho Count Per Rotation, a constant that sets any amount of rotations to the tacho_count used by the motor.


        // forward function using vehicle
        public void forward(double rotations, Robot robot)
        {
            // resets the tacho count to 0 for both motors.
            motorL.ResetTacho();
            motorR.ResetTacho();

            // gets the tacho count for the forward amount.
            double tacho_count = tcpr * rotations;

            // sets a wait handle used to wait until the robot has finished moving forward
            WaitHandle wait_event = Robot_Vehicle.Forward(speed, ((uint)tacho_count), true);
            wait_event.WaitOne(); // waits till the robot has finished moving forward.
            
            // turns the motors off.
            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        public void backward(double rotations, Robot robot)
        {
            // resets the tacho count to 0 for both motors.
            motorL.ResetTacho();
            motorR.ResetTacho();

            // gets the tacho count for the forward amount.
            double tacho_count = tcpr * rotations;

            // sets a wait handle used to wait until the robot has finished moving backward
            WaitHandle wait_event = Robot_Vehicle.Backward(speed, ((uint)tacho_count), true);
            wait_event.WaitOne(); // waits till the robot has finished moving forward.

            // turns the motors off.
            Robot_Vehicle.Off();
            /// source -> https://github.com/Larsjep/monoev3/blob/release/VehicleExample/Program.cs
            /// Code is heavily changed but inspired by this example.
        }

        // do not input 360 or 0 degrees due to loop back error
        /// <summary>
        /// Rotates The robot
        /// </summary>
        /// <param name="degrees"> the amount of degrees the robot should turn </param>
        /// <param name="robot"> the robot class </param>
        /// <param name="dir"> true turns the robot from right to left and false from left to right</param>
        public void Rotate(double degrees, Robot robot, bool dir = true)
        {
            // if the robot turns from right to left, the gyro sensor returns negative degree values.
            // thus if the degrees are less than 0 but the gyro sensor is returning positive values,
            // the degrees are set to a positive amount.
            if(degrees < 0 && !dir)
            {
                degrees = 360 + degrees;
            }

            // the button event to stop the robot from rotating
            ButtonEvents buts = new ButtonEvents();
            bool done = false; // stop variable for the while loop

            // resets the gyro sensor
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
            // sets the button event to set the value of done to true when the escape button is pressed
            buts.EscapePressed += () => {
                done = true;
            };

            double new_turned = 0; // needed because of when vsensor is called after the while loop it will add more degrees than the currently turned amount ( seems to be a bug in the libraries )
            if (degrees < 0) // determines if the rotation is from right to left
            {
                // prints the amount rotated atleast once to the lcd, and sets the new_turned variable to the reading of the vsensor
                do
                {
                    vars.print($"diff to:  {vsensor.read_gyro_r()}");
                    new_turned = vsensor.read_gyro_r();
                }
                while (vsensor.read_gyro_r() >= degrees && !done);
            }
            else // the rotation is from left to right
            {
                // prints the amount rotated atleast once to the lcd, and sets the new_turned variable to the reading of the vsensor
                do
                {
                    vars.print($"diff to: {vsensor.read_gyro_r()}");
                    new_turned = vsensor.read_gyro_r();
                }
                while (vsensor.read_gyro_r() <= degrees && !done);
            }
            // brakes the motors after turning to X degrees
            motorL.Brake();
            motorR.Brake();
            // determines the new angle of the robot relative to the original position in the base.
            double curr_turned = robot.angle;
            double new_angle = curr_turned + new_turned;

            // keeps the currently turned angle less than 180 and bigger than -180. this is to calculate the quickest way to turn next time it needs to rotate.
            if(new_angle > 180)
            {
                new_angle = new_angle - 360;
            }
            if (new_angle < -180)
            {
                new_angle = new_angle + 360;
            }

            // sets the new robot angle.
            robot.angle = robot.angle + new_turned;
        }

        /// <summary>
        /// moves the arm of the robot up and down
        /// </summary>
        /// <param name="degrees"> the amount of degrees the arm should turn </param>
        /// <param name="speed"> the speed the arm should turn</param>
        /// <param name="down"> if the arm should also go back down after going up </param>
        /// <param name="time"> the time between going up and down </param>
        public void MoveArm(double degrees, int speed = 100, bool down = false, int time = 0)
        {
            // this is used to also be able to move the arm down, when moving the arm down, the speed is negative, but the degrees are still positive.
            if (degrees < 0)
            {
                speed = speed * -1;
                degrees = degrees * -1;
            }
            // resets the tacho count to 0
            motorArm.ResetTacho();

            // sets a wait handle to wait untill the arm has finished moving up or down.
            WaitHandle WaitHandle_up = motorArm.PowerProfile((sbyte)speed, 0, (uint)degrees, 0, true);
            WaitHandle_up.WaitOne(); // waits untill the arm has finished moving.
            // if the arm should also move down, it sets the speed to a negative, and repeats the previous process.
            // NOTE: only the speed is set to negative because the previous code handling for if the arm should move down is still active,
            // so if the arm was going down it will now, instead, go up. this is just a reverse of the previous arm movement.
            if (down)
            {
                speed = speed * -1;
                Thread.Sleep(time);
                WaitHandle WaitHandle_down = motorArm.PowerProfile((sbyte)speed, 0, (uint)(degrees), 0, true);
                WaitHandle_down.WaitOne();
            }

            // turns the motor of the arm off.
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
        public double[] pos = new double[2];
        public double angle;
        public string name;

        // constructor
        /// <summary>
        /// constructor for the challenge class
        /// </summary>
        /// <param name="x">x position of the challenge </param>
        /// <param name="y">y position of the challenge </param>
        /// <param name="angle"> angle the robot should rotate to when position is reached </param>
        /// <param name="name"> name of the challenge </param>
        public Challenge(double x, double y, double angle, string name)
        {
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
            new Challenge(14, 95, 0, "M1"),
            new Challenge(94, 75, 0, "M4"),
            new Challenge(71.5, 45, -90, "M5"),
            new Challenge(172, 101.5, -180, "M8"),
            new Challenge(129, 101.5, 0, "M9"),
            new Challenge(153, 77.5, 0, "M10"),
            new Challenge(112, 30, 0, "M6"),
            
        };
        // the nodes the robot can drive towards.
        public Challenge[] Nodes = new Challenge[]
        {
            new Challenge(44.5, 25.5, 0, "basis"),
            new Challenge(92, 44.5, 0, "N1_toM4"),
            new Challenge(54, 80, 0, "N2_fromM4"),
            new Challenge(129, 25, 0, "N3_toN4"),
            new Challenge(162, 42, 0, "N4_toN5"),
            new Challenge(159, 61, 0, "N4_toDUWCHALL")
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
            double rc = dy / dx;                                                        // rc needed to get rotation to face destination ( atan(rc) = degrees relative to x-axis
            double angle;
            if (rc > 0) { angle = (Math.Atan(rc) * (180 / Math.PI)) * -1; }             // getting angle to face destination
            else { angle = 180 + (Math.Atan(rc) * (180 / Math.PI)) * -1; }

            
            angle = (angle - robot.angle);                                              // gets the difference between new angle and current angle and sets that as rotation angle
            if (reverse_rotate)
            {
                angle = -180 + angle;
            }
            
            this.rotate(angle, robot);                                                  // rotates to destination
            double[] result = { dx, dy };                                               // returns A and B side of triangle

            return result;
        }

        // go to specified challenge
        /// <summary>
        /// goes to a challenge specified bij challenge param
        /// </summary>
        /// <param name="challenge"> the challenge to go to </param>
        /// <param name="robot"> the robot object </param>
        /// <param name="wait"> the ResetEvent for when the robot has finished driving </param>
        /// <param name="rotate_at_end"> for if the robot should rotate to challenge rotation when challenged reached</param>
        /// <param name="reverse_rotate"> forces the angle of the robot to rotate to a negative version of that angle. </param>
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
            pos = challenge.pos;                                                        // sets the position of the robot to that of the challenge, this assumes the robot has reached the challenge
            if (rotate_at_end) { this.rotate_at_end(challenge, robot); }                // rotates the robot to specified angle when robot has reached the challenge
            wait.Set();                                                                 // sets the AutoResetEvent to signal that the robot has finished moving.
        }

        // rotates to the challenge when arrived at the designated challenge stop point
        public void rotate_at_end(Challenge challenge, Robot robot)
        {
            angle = (challenge.angle - this.angle);                                 // gets the difference between challenge angle and current angle and sets that as rotation angle
            this.rotate(angle, robot);                                              // rotates specified angle.
        }
    }


    /// MAIN
    class MainClass
    {
        public static void Main(string[] args)
        {

            Robot robot = new Robot();                                  // initializes the robot object

            V_Motor vmotor = new V_Motor();                             // creates the vmotor object for forced movement
            V_Sensor vsensor = new V_Sensor();                          // creates the vsensor object for forced sensor readings

            AutoResetEvent wait = new AutoResetEvent(false);            // wait event used to check if the robot has finished moving
            ButtonEvents buts = new ButtonEvents();                     // button event object

            // event to force signal the wait object.
            buts.EnterPressed += () => {
                wait.Set(); 
            };

            robot.goto_chall(robot.Nodes[3], robot, wait, false, true);
            wait.WaitOne(); wait.Reset();
            robot.goto_chall(robot.Nodes[4], robot, wait, false);
            wait.WaitOne(); wait.Reset();
            robot.goto_chall(robot.Nodes[5], robot, wait, false);
            wait.WaitOne(); wait.Reset();

            vmotor.forward(1.5, robot);
            vmotor.backward(0.7, robot);

            robot.rotate(-35, robot);

            robot.goto_chall(robot.Nodes[2], robot, wait, false);

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
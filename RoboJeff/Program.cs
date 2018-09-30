using System;
using MonoBrickFirmware;
using MonoBrickFirmware.Display.Dialogs;
using MonoBrickFirmware.Display;
using MonoBrickFirmware.Movement;
using MonoBrickFirmware.Sensors;
using MonoBrickFirmware.UserInput;
using MonoBrickFirmware.Sound;
using System.Threading;

/* 
 * 
 * 
 * 
 * 
 */

namespace RoboJeff
{
    class MainClass
    {
        public class variables
        {
            public Font f = Font.SmallFont;
            public Point offset = new Point(0, 12);
            public static Point p = new Point(10, Lcd.Height - 75);
            public static Point boxSize = new Point(100, 24);
            public Rectangle box = new Rectangle(p, p + boxSize);
            public EV3UltrasonicSensor US = new EV3UltrasonicSensor(SensorPort.In1);
            public EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.In2, GyroMode.Angle);
            public EV3TouchSensor touch = new EV3TouchSensor(SensorPort.In3);
            public EV3ColorSensor color = new EV3ColorSensor(SensorPort.In4);
            /// source -> https://github.com/Larsjep/monoev3/blob/release/LcdExample/Program.cs
        }

        public static bool done = false;

        public static void Main(string[] args)
        {
            variables vars = new variables();
            //Speaker speaker = new Speaker (50);

            LcdConsole.WriteLine("Herro");

            ManualResetEvent terminateProgram = new ManualResetEvent(false);
            ButtonEvents buts = new ButtonEvents();
            int i = 0;
            while (!done)
            {
                buts.EscapePressed += () => {
                    done = true;
                    terminateProgram.Set();
                };
                buts.UpPressed += () => {
                    LcdConsole.WriteLine("Beep");
                };
                buts.DownPressed += () => {
                    LcdConsole.WriteLine("Buzz");
                };
                buts.EnterPressed += () => {
                    LcdConsole.WriteLine("enter");
                };
                read_degree(vars, i);
                i++;
            }

            terminateProgram.WaitOne();

            /// source -> https://github.com/Larsjep/monoev3/blob/release/SoundExample/Program.cs
        }


        public static void read_degree(variables vars, int i)
        {
            Lcd.Clear();
            Lcd.WriteTextBox(vars.f, vars.box + vars.offset * 0, "US: " + vars.US.Read().ToString() + " " + i.ToString(), true);
            Lcd.WriteTextBox(vars.f, vars.box + vars.offset * 1, "Gyro: " + vars.gyro.Read().ToString() + " " + i.ToString(), true);
            Lcd.WriteTextBox(vars.f, vars.box + vars.offset * 2, "Touch: " + vars.touch.Read().ToString() + " " + i.ToString(), true);
            Lcd.WriteTextBox(vars.f, vars.box + vars.offset * 3, "Color: " + vars.color.Read().ToString() + " " + i.ToString(), true);
            Lcd.Update();

            /// source -> https://github.com/Larsjep/monoev3/blob/release/LcdExample/Program.cs
        }
    }
}
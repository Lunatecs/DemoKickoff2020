/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainConstants {
        //Right motors
        public static final int Right_Front_ID = 6;
        public static final int Right_Middle_ID = 5;
        public static final int Right_Back_ID = 4;
        //Left motors
        public static final int Left_Front_ID = 3;
        public static final int Left_Middle_ID = 2;
        public static final int Left_Back_ID = 1;

        public static final int Ultrasonic_Ping_ID = 8;
        public static final int Ultrasonic_Echo_ID = 9;

        public static final double TrackingProportional = 0.07;
        public static final double TrackingIntegral = 0.025;
        public static final double TrackingDerivative = 0.005;

        public static final double WallProportional = 0.6;
        public static final double WallIntegral = 2.4;
        public static final double WallDerivative = 0.0375;
    }
    
    public static final class ControllerConstants {
        public static int Joystick_USB_Driver = 0;
        public static int Joystick_USB_Operator = 1;

        public static int Joystick_Right_X_Axis = 4;
        public static int Joystick_Right_Y_Axis = 5;
        public static int Joystick_Left_X_Axis = 0;
        public static int Joystick_Left_Y_Axis = 1;

        public static int Red_Button_ID = 2;
        public static int Green_Button_ID = 1;
        public static int Yellow_Button_ID = 4;
        public static int Blue_Button_ID = 3;
    }
}

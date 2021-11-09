// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static final double kPi = 3.14159265359;

    public static class DRIVE
    {
        public static int leftMasterCANID = 2;
        public static int rightMasterCANID = 0;
        public static int leftMinionCANID = 10;
        public static int rightMinionCANID = 9;

        public static final double kWheelDiameterInches = 5.9575;
        public static final double kWheelCircumference = kWheelDiameterInches * kPi;

        public static double kP = 1;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;

        public static double driveOpenRampRate = 0.5;
        public static double driveCloseRampRate = 0.5;
    }

    public static class SHOOT {
        
        public static int shootRightCANID = 7;
        public static int shootLeftCANID = 5;
        public static int shootHoodCANID = 8; 
         
        
    }
}

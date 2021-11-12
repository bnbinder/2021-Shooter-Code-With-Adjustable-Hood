// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static double kPi = 3.14159265359;

    public static class DRIVE
    {
        public static int leftMasterCANID = 10;
        public static int rightMasterCANID = 0;
        public static int leftMinionCANID = 2;
        public static int rightMinionCANID = 9;

        public static double kWheelDiameterInches = 5.9575;
        public static double kWheelCircumference = kWheelDiameterInches * kPi;

        public static double kP = 1;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0; //! (%ofMotor x 1023) / maxNativeVelocity

        public static double driveOpenRampRate = 0.5;
        public static double driveCloseRampRate = 0.5;

        public static double oneFullRotation = 1023;
        public static double maxNativeVelocity = 2084; //TODO make sure this is right for talon fx
    }

    public static class SHOOT {
        
        public static int shootRightCANID = 7;
        public static int shootLeftCANID = 5;
        public static int shootHoodCANID = 8; 
         
        public static double kP = 1;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0; //! (%ofMotor x 1023) / maxNativeVelocity
        
        public static double shootOpenRampRate = 0.1;
        public static double shootCloseRampRate = 0.1;

        public static double oneFullRotation = 1023;
        public static double maxNativeVelocity = 2084; //TODO make sure this is right for talon fx
    }
}

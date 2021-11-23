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

        public static double maxAutoVelocity = 0;
        public static double maxAutoAcceleration = 0;
        //TODO find and set these
         
        public static double kP = 1;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0; //! (%ofMotor x 1023) / maxNativeVelocity this also might be wrong
        
        public static double shootOpenRampRate = 0.5;
        public static double shootCloseRampRate = 0.5;
        public static double shootHoodOpenRamp = 0.1;
        public static double shootHoodCloseRamp = 0.1;

        public static double oneFullRotationPIDF = 1023; //when the fuck will i use this
        public static double maxNativeVelocity = 2084; //TODO make sure this is right for talon fx

        public static double hoodMaxVel = 5; //TODO get these
        public static double hoodMaxAccel = 0;

        //TODO dont change P unless
        //TODO increase D slowly 
        public static double hoodKP = 0.00005; //0.00010;
                                        //0.00005
        public static double hoodKI = 0.000000; //or 1
        public static double hoodKD = 0.00000023; //0.000000017;
        public static double hoodKF = 0;
    }

    public static class VISION
    {
        //? should have made hood and shoot class seperate, too late now this houses all the hood classes starting now 11/19/21

                          //!  lol
        public static double greerRatioLilToBigOne = 10/60; //TODO theres more lol 
        public static double greerRatioLilToBigTwo = 14/52;
        public static double totalGreerRatio = 22.2857142857; //amount little has to spin for one big
        public static double hoodLength = 18.145386; //TODO find this bullshit (prob on cad)
        public static double heightGoal = 89.75;
        public static double heigtCameraAtRest = 0; //!may not need this //at rest on actual 3d printed part lol
        public static double angleNativeCameraAtRest = 18; //on 3d print part
        public static double BindersConstant = 1; //:tro:

        public static double minimumShitSpeed = 0; //at hood rest (or minimum degrees/rad/native if not good enough)
        public static double minimumShitHood = 0;

        public static double setPoint0 = 600; //TODO need to get more accurate thing
        //set point 700 moves arm up to 25 native units (basically 0)
        //with respect to current P and D values (0.00005 and 0.00000023)
        public static double maxRotate = 4000; //TODO need to stop being a bitch and make this shit higher
    }
}

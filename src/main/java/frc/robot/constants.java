// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static final double kPi = 3.14159265359;

    public static class DRIVE
    {
        public static int leftMasterCANID = 0;
        public static int rightMasterCANID = 0;
        public static int leftMinionCANID = 0;
        public static int rightMinionCANID = 0;

        public static final double kWheelDiameterInches = 5.9575;
        public static final double kWheelCircumference = kWheelDiameterInches * kPi;
    }
}

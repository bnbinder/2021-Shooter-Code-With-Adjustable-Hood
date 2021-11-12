// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.SHOOT;

/** Add your docs here. */
public class Limelight {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry horizonAngle = table.getEntry("tx");
    private NetworkTableEntry verticAngle = table.getEntry("ty");
    private NetworkTableEntry seeTarget = table.getEntry("tv");
    private NetworkTableEntry areaOfTarget = table.getEntry("ta");
    private NetworkTableEntry pipeline = table.getEntry("pipeline");
    private TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints();
    private PIDController shootController = new PIDController(SHOOT.kP, SHOOT.kI, SHOOT.kD);

    private Limelight()
    {
        pipeline.setValue(0);
    }

    public void updateAutoShoot()
    {
        shootController.
    }
}

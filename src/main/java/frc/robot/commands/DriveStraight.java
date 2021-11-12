// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Drive;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraight extends Command {
  private double dist;
  public DriveStraight(double distance) {
    dist = distance;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Drive.getInstance().setMagic(dist);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Drive.getInstance().motionMagical();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Drive.getInstance().isMotionDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}

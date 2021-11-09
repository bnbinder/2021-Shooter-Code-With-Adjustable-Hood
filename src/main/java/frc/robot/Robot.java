// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Drive mDrive = Drive.getInstance();

  private Timer LeTimer = new Timer();
  private Timer endTime = new Timer();

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    mDrive.setMagic(12);
    LeTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    if(mDrive.isMotionDone() == false)
    {
      mDrive.motionMagical();
    }
    mDrive.updateDrive();
    SmartDashboard.putNumber("time", LeTimer.get());
  }

  @Override
  public void disabledInit() {
    LeTimer.stop();
    //this is swerdlow idea, very useful
    endTime.start();

  }

  @Override
  public void disabledPeriodic() {
    if(endTime.get() > 1.5)
    {
      //do stuff
      endTime.stop();
    }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}

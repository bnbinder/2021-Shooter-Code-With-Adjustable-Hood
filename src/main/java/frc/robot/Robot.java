// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drive;
import frc.robot.commands.DriveStr8;

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
  private Shooter mShoot = Shooter.getInstance();

  private double var;

  private Timer LeTimer = new Timer();
  private Timer endTime = new Timer();

  private Command autoCommand;

  private XboxController xbox = new XboxController(0);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    autoCommand = new DriveStr8();
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    updateSensors();
  }

  @Override
  public void teleopInit() {
    //mDrive.setMagic(12);
    LeTimer.start();
    mDrive.resetOdo();
    mShoot.setHoodPos(0);
   // XYPlane.getInstance().resetShit();
    mShoot.resetPID();
  }

  @Override
  public void teleopPeriodic() 
  {
    if(xbox.getRawAxis(2) > 0)
    {
      mDrive.setDrivePercent(xbox.getRawAxis(2), xbox.getRawAxis(2));
    }
    else if(xbox.getRawAxis(3) > 0)
    {
      mDrive.setDrivePercent(-xbox.getRawAxis(3), -xbox.getRawAxis(3));
    }
    else
    {
      mDrive.setDrivePercent(0, 0);
    }


    if(xbox.getRawAxis(1) > 0)
    {
      //mShoot.shootPercent(xbox.getRawAxis(1));
    }
    else if(xbox.getAButton())
    {
      var = mShoot.shootCalculateShit(4000);
      if(var > .15)
      {
        var = 0.15;
      }
      mShoot.shootHoodPercent(var);
    }
    else if(xbox.getBButton())
    {
      var = mShoot.shootCalculateShit(2000);
      if(var > .15)
      {
        var = 0.15;
      }
      mShoot.shootHoodPercent(var);
     // XYPlane.getInstance().changebastatrsSensro();
    }
    else
    {
      mShoot.shootPercent(0);
      mShoot.shootHoodPercent(0);
    }



    if(mDrive.isMotionDone() == false)
    {
     // mDrive.motionMagical();
    }
    SmartDashboard.putNumber("ijfhrughhgjrhbgrbihjdijdb", mShoot.shootCalculateShit(3000));
    SmartDashboard.putNumber("thechjeetos bonless wings", mShoot.shootCalculateShit(4000));
    SmartDashboard.putNumber("var", var);
    updateSensors();
    //XYPlane.getInstance().fuck();
    //XYPlane.getInstance().PIDset(XYPlane.getInstance().fuckyou());
    SmartDashboard.putBoolean("abut", xbox.getAButton());
    SmartDashboard.putNumber("time", LeTimer.get());
    
  }

  @Override
  public void disabledInit() {
    LeTimer.stop();
    //this is swerdlow idea, very useful
    endTime.start();
    mDrive.resetOdo();

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

  public void updateSensors()
  {
    mDrive.updateDrive();
    mShoot.updateShoot();
  }
}

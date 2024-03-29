// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drive;
import frc.robot.Constants.SHOOT;
import frc.robot.Constants.VISION;
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
  private Limelight mLimelight = Limelight.getInstance();

  private double shootff;

  private boolean pressA = false;

  private double leftOut;
  private double rightOut;
  private double leftTurn;
  private double rightTurn;
  
  private double var = 0;

  private Timer LeTimer = new Timer();
  private Timer endTime = new Timer();

  private Command autoCommand;

  private double max; //slider number
  private double realPower; //calculate the real power 
  //needed because this cant be changed or var will be changed since var has limit and this dont
  //i think, or im just dumb and am using two vars when only one is needed
 

  private XboxController xbox = new XboxController(0);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() { //i should delete this before i accidentally press auto
    autoCommand = new DriveStr8();
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    updateSensors();
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("maxx", 0);
    

    //mDrive.setMagic(12);
    LeTimer.start();
    mDrive.resetOdo();
    mShoot.setHoodPos(0);
   // XYPlane.getInstance().resetShit();
    mShoot.resetPID();
    mShoot.shootHoodPercent(0);
    pressA = false;
  }

  @Override
  public void teleopPeriodic() 
  {

    if(Math.abs(xbox.getRawAxis(0)) > 0.1)
    {
      leftTurn = 0;//xbox.getRawAxis(0);
      rightTurn = 0;//xbox.getRawAxis(0); //TODO find out what stick controls turn
    }
    else
    {
      leftTurn = 0;
      rightTurn = 0;
    }

    leftOut = xbox.getRawAxis(2) - xbox.getRawAxis(3);
    rightOut = xbox.getRawAxis(2) - xbox.getRawAxis(3);


    
    mDrive.setDrivePercent(leftOut + (leftTurn/2), rightOut - (rightTurn/2));

    //leftOut = -xbox.getRawAxis(3);
    //rightOut = -xbox.getRawAxis(3);
    //mDrive.setDrivePercent(leftOut, rightOut);


    if(xbox.getRawAxis(1) > 0.1)
    {
      shootff = mShoot.ffshoot(xbox.getRawAxis(1) * SHOOT.maxNativeVelocity);
      mShoot.shootVelocity(shootff + (xbox.getRawAxis(1) * SHOOT.maxNativeVelocity));
      //TODO test this tommorrow or sometime
    }
    /*
    else if(xbox.getBButton())
    {
      var = realPower;
      if(var > .15)
      {
        var = 0.15;
      }
      mShoot.shootHoodPercent(var);
     // XYPlane.getInstance().changebastatrsSensro();
    }
    */
    else
    {
      mShoot.shootPercent(0);
      //mShoot.shootHoodPercent(0);
      mShoot.resetKI();
      var = 0;
      //realPower = 0;
    }

    if(xbox.getAButtonPressed())
    {
      if(xbox.getAButtonReleased())
      {
        pressA = !pressA;
      }
    }

    if(pressA)
    {
      //var = mShoot.shootCalculateShit(1000); //4000
      var = realPower;
      if(var > .17)
      {
        var = 0.17;
      }
      mShoot.shootHoodPercent(var);
    }
    else
    {
      mShoot.shootHoodPercent(0);
    }


    if(mDrive.isMotionDone() == false)
    {
     // mDrive.motionMagical();
    }
    SmartDashboard.putNumber("ffshoot", shootff);
    SmartDashboard.putNumber("ff", mShoot.ffHood(max));
    //SmartDashboard.putNumber("getCamHit", mShoot.getCameraHeight());
    //SmartDashboard.putNumber("getCamAng", mShoot.getCameraAngle());
    SmartDashboard.putNumber("var", var);
    SmartDashboard.putNumber("shoot ereor", mShoot.geterror());
    SmartDashboard.putNumber("shoottvelo eroer", mShoot.getveleror());
    updateSensors();
    //XYPlane.getInstance().fuck();
    //XYPlane.getInstance().PIDset(XYPlane.getInstance().fuckyou());
    SmartDashboard.putBoolean("abut", xbox.getAButton());
    SmartDashboard.putBoolean("a", pressA);

    
    max = SmartDashboard.getNumber("maxx", 0);
    SmartDashboard.putNumber("the thing", mShoot.shootCalculateShit(max));
    realPower = SmartDashboard.getNumber("the thing", 0);
    SmartDashboard.putNumber("imsoscard", realPower);
    
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
    mLimelight.updateAutoShoot();
  }
}

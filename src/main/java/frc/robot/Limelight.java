// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.SHOOT;
import frc.robot.Constants.VISION;

/** Add your docs here. */
public class Limelight {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry horizonAngle = table.getEntry("tx");
    private NetworkTableEntry verticAngle = table.getEntry("ty");
    private NetworkTableEntry seeTarg = table.getEntry("tv");
    private NetworkTableEntry areaOfTarget = table.getEntry("ta");
    private NetworkTableEntry pipeline = table.getEntry("pipeline");
    private TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints();
    private PIDController shootController = new PIDController(SHOOT.kP, SHOOT.kI, SHOOT.kD);
    private Drive mDrive = Drive.getInstance();
    private Shooter mShoot = Shooter.getInstance();
    private double complementAngy;
    private double horizonTX, verticTY, distance, hoodPos, shitSpeed, hoodDGAIN;
    private boolean seeTarget;
    private boolean stuff = true;

    private Limelight()
    {
        pipeline.setValue(0);
    }

    
    public static Limelight getInstance()
    {
        return InstanceHolder.mInstance;
    }


    public void updateAutoShoot()
    {
        horizonTX = horizonAngle.getDouble(0.0);
        verticTY = verticAngle.getDouble(0.0);
        distance = getDistance();
        seeTarget = MkUtil.doubleToBoolean(seeTarg.getDouble(0.0));
        SmartDashboard.putNumber("hoodPos", hoodPos);
        SmartDashboard.putNumber("y", verticTY);
        SmartDashboard.putBoolean("verig", stuff);
        SmartDashboard.putNumber("distance", getDistance());

        autoHood();
    }

    public void autoHood()
    {
        if(!seeTarget)
        {
            //mShoot.shootHoodPercent(mShoot.shootCalculateShit(4000));
            hoodPos = mShoot.getHoodSensorPos();
            stuff = false;
        }
        else if(seeTarget && Math.abs(verticTY) <= VISION.limelightThreshold)
        {
            //mShoot.shootHoodPercent(mShoot.shootCalculateShit(hoodPos));
            stuff = true;
        }
        else if(seeTarget  && Math.abs(verticTY) > VISION.limelightThreshold)
        {
            //mShoot.shootHoodPercent(verticTY * -1);
            hoodPos = mShoot.getHoodSensorPos();
        }
    }

    //!      i am so fucking funny laugh
    public void autoShit()
    {
        //may need sine or cosine equation depending on needs of auto shoot velocity
    }


    public double getDistance()
    {
        // d = distance, h2 = height of goal, h1 = heihgt of camera, a1 = angle from robot to camera, a2 = angle from camera to target / error
        //d = (h2-h1) / tan(a1+a2)
        //TODO i hope ty gives us right angle, also need to test the nativeToDegree i created
        //TODO also see how to get a2 correctly, dont know if this is the correct way
        
        //TODO while i can account for the y movement of the camera, i need to account for the x movement of the camer
        //TODO work on this after working on the height and angle shit
        complementAngy = 90 - mShoot.getCameraAngle();
        return (VISION.heightGoal - mShoot.getCameraHeight()) / 
                Math.tan(MkUtil.degreesToRadian(complementAngy + verticTY));
        //! subtract the positive x value that the camera is at; camera is up, its x amount away from x position of starting posititon
    }

        
    private static class InstanceHolder
    {
        private static final Limelight mInstance = new Limelight();
    } 
    
}

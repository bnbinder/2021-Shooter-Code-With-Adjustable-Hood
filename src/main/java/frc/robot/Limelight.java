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
import frc.robot.Constants.VISION;

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
    private Drive mDrive = Drive.getInstance();
    private Shooter mShoot = Shooter.getInstance();
    private double complementAngy;
    private double horizonTX, verticTY, distance, hoodPos, shitSpeed;

    private Limelight()
    {
        pipeline.setValue(0);
    }

    public void updateAutoShoot()
    {
        horizonTX = horizonAngle.getDouble(0.0);
        verticTY = verticAngle.getDouble(0.0);
        distance = getDistance();
        //autohood();
    }

    public void autoHood()
    {
        hoodPos = distance * VISION.BindersConstant; //my constant lol
        //!mShoot.shootHoodPercent(mShoot.shootCalculateShit(hoodPos));
    }

    //!      i am so fucking funny laugh
    public void autoShit()
    {
        
    }


    public double getDistance()
    {
        // d = distance, h2 = height of goal, h1 = heihgt of camera, a1 = angle from robot to camera, a2 = angle from camera to target / error
        //d = (h2-h1) / tan(a1+a2)
        //TODO i hope ty gives us right angle, also need to test the nativeToDegree i created
        //TODO also see how to get a2 correctly, dont know if this is the correct way
        complementAngy = 180 - mShoot.getCameraAngle();
        return (mShoot.getCameraHeight() - VISION.heightGoal) / 
                        Math.tan(complementAngy + verticTY);
    }
}

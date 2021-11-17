// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** Add your docs here. */
public class XYPlane {
    //!not needed, only needed for swerve drive (not need for now, at least)
    private double x;
    private double y;
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(100,100);
    private ProfiledPIDController fuckPID = new ProfiledPIDController(1, 0, 0,constraints);
    private double fuckSensor = 0;
    public static XYPlane getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void resetShit()
    {
        fuckPID.reset(0, 0);
        fuckSensor = 0;
    }
    public void PIDset(double param)
    {
        SmartDashboard.putNumber("fuckyou", fuckPID.calculate(fuckSensor, 200));
        SmartDashboard.putNumber("fuckingsensor", fuckSensor);
    }

    public void changeFuckingSensor()
    {
        fuckSensor +=1;
    }

    public void changebastatrsSensro()
    {
        fuckSensor -=1;
    }
    
    public double fuckyou()
    {
        return fuckSensor;
    }

    public void fuck()
    {
        fuckSensor += fuckPID.calculate(fuckSensor,200);
    }


















    public double getX()
    {
        return x;
    }
    public double getY()
    {
        return y;
    }
    public void setX(double xParam)
    {
        x = xParam;
    }
    public void setY(double yParam)
    {
        y = yParam;
    }
    public void addX(double addX)
    {
        x += addX;
    }
    public void addY(double addY)
    {
        y += addY;
    }

    private static class InstanceHolder
    {
        private static final XYPlane mInstance = new XYPlane();
    } 
}

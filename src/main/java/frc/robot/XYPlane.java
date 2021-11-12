// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class XYPlane {
    //!not needed, only needed for swerve drive (not need for now, at least)
    private double x;
    private double y;
    public static XYPlane getInstance()
    {
        return InstanceHolder.mInstance;
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

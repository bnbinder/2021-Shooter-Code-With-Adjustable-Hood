// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SHOOT;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class Shooter {
    TalonFX ShootRight = new TalonFX(SHOOT.shootRightCANID);
    TalonFX ShootLeft= new TalonFX(SHOOT.shootLeftCANID);
    TalonFX ShootHood = new TalonFX(SHOOT.shootHoodCANID);

    private double leftShootPosNative, rightShootPosNative, 
                   leftShootPosInch, rightShootPosInch, 
                   leftShootVelInch, rightShootVelInch, 
                   leftShootVelNative, rightShootVelNative, 
                   averageShootNativeDistance, averageShootInchesDistance, 
                   averageShootNativePer100ms, averageShootInchesPerSec;

    private Shooter()
    {
        ShootLeft.configFactoryDefault();
        ShootRight.configFactoryDefault();
        ShootHood.configFactoryDefault();
    
        ShootHood.setNeutralMode(NeutralMode.Coast);
        ShootRight.setNeutralMode(NeutralMode.Coast);
        ShootLeft.setNeutralMode(NeutralMode.Coast);
        
        ShootLeft.config_kP(0, SHOOT.kP);
        ShootLeft.config_kI(0, SHOOT.kI);
        ShootLeft.config_kD(0, SHOOT.kD);
        ShootLeft.config_kF(0, SHOOT.kF);

        ShootRight.config_kP(0, SHOOT.kP);
        ShootRight.config_kI(0, SHOOT.kI);
        ShootRight.config_kD(0, SHOOT.kD);
        ShootRight.config_kF(0, SHOOT.kF);
        
    }
    
    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateShoot()
    {
        ShootLeft.setInverted(true);

        leftShootPosNative = ShootLeft.getSelectedSensorPosition();
        rightShootPosNative = ShootRight.getSelectedSensorPosition();
        averageShootNativeDistance = (leftShootPosNative + rightShootPosNative) / 2.0;

        leftShootPosInch = MkUtil.nativeToInches(leftShootPosNative);
        rightShootPosInch = MkUtil.nativeToInches(rightShootPosNative);
        averageShootInchesDistance = (leftShootPosInch + rightShootPosInch) / 2.0;

        leftShootVelNative = ShootLeft.getSelectedSensorVelocity();
        rightShootVelNative = ShootRight.getSelectedSensorVelocity();
        averageShootNativePer100ms = (leftShootVelNative + rightShootVelNative) / 2.0;

        leftShootVelInch = MkUtil.nativePer100MstoInchesPerSec(leftShootVelNative);
        rightShootVelInch = MkUtil.nativePer100MstoInchesPerSec(rightShootVelNative);
        averageShootInchesPerSec = (leftShootVelInch + rightShootVelInch) / 2.0;
    }

    public void shootPercent(double shootR, double shootL)
    {
        ShootLeft.set(ControlMode.PercentOutput, shootL);
        ShootRight.set(ControlMode.PercentOutput, shootR);
    }

    public void shootVelocity(double vel)
    {
        ShootLeft.set(ControlMode.Velocity, vel);
    }
    
    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVE;

/** Add your docs here. */
public class Drive {
    TalonFX leftMaster = new TalonFX(DRIVE.leftMasterCANID);
    TalonFX rightMaster = new TalonFX(DRIVE.rightMasterCANID);
    TalonFX leftMinion = new TalonFX(DRIVE.leftMinionCANID);
    TalonFX rightMinion = new TalonFX(DRIVE.rightMinionCANID);
    AHRS navX = new AHRS();
    private double distance; 
    private double leftPosNative, rightPosNative, 
                   leftPosInch, rightPosInch, 
                   leftVelInch, rightVelInch, 
                   leftVelNative, rightVelNative, 
                   averageNativeDistance, averageInchesDistance, 
                   averageNativePer100ms, averageInchesPerSec;
    private Drive()
    {
        //sets factory defualt, good?
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightMinion.configFactoryDefault();
        leftMinion.configFactoryDefault();
    
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        leftMinion.setNeutralMode(NeutralMode.Coast);
        rightMinion.setNeutralMode(NeutralMode.Coast);

        leftMaster.config_kP(0, DRIVE.kP);
        leftMaster.config_kI(0, DRIVE.kI);
        leftMaster.config_kD(0, DRIVE.kD);
        leftMaster.config_kF(0, DRIVE.kF);

        rightMaster.config_kP(0, DRIVE.kP);
        rightMaster.config_kI(0, DRIVE.kI);
        rightMaster.config_kD(0, DRIVE.kD);
        rightMaster.config_kF(0, DRIVE.kF);

        //gets velocity in past 10ms when determining what velocity to have (motion magic)
        //!leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

        //how many velocities to compare/average?
        //?     test later
        //TODO leftMaster.configVelocityMeasurementWindow(32);

        //what this do?
        //?     test later
        //!leftMaster.configAllowableClosedloopError(0, 1);

        //makes sure motors dont go above 1 (100%)?
        //?     test later
        //!leftMaster.configClosedLoopPeakOutput(0, 1);

        //when input is below deadband but above/below zero, motors dont move
        //see documentation
        //?     test later
        //!leftMaster.configNeutralDeadband(0.01);

        //what this do?
        //motor not go below this number when forward?
        //?     test later
        //!leftMaster.configNominalOutputForward(0);

        //what this do?
        //motor not go below this number when backward?
        //?     test later
        //!leftMaster.configNominalOutputReverse(0);

        //during open loop, 0.5 seconds before motor go from 0 to selected output
        //?     test later
        //!leftMaster.configOpenloopRamp(DRIVE.driveOpenRampRate);

        //during close loop, 0.5 seconds before motor go from 0 to selected output
        //?     test later
        //!leftMaster.configClosedloopRamp(DRIVE.driveCloseRampRate);

        //sets pid fI to set number, but what is pidIdx?
        //?     research
        //!leftMaster.setIntegralAccumulator(iaccum, pidIdx, timeoutMs));
        //!leftMaster.setIntegralAccumulator(iaccum);

        //if motor takes longer than 20ms to get from start to finish, can bus not utilizing enough and stuff wrong?
        //?     research
        //!leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        //!leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        //sets time (ms) of how long one loop in closed loop is?
        //?     research
        //!leftMaster.configClosedLoopPeriod(0,1);

        //limits motor output (how much motor uses)?
        //?     research
        //!leftMaster.configStatorCurrentLimit();

        //limits motor input (how much motor takes)?
        //?     research
        //!leftMaster.configSupplyCurrentLimit();

        //enable and set the max amount of voltage a motor can use?
        //?     research
        //!leftMaster.enableVoltageCompensation(true);
        //!leftMaster.configVoltageCompSaturation(10);

        //peak output 10% forward and back?
        //?     research and test
        //!leftMaster.configPeakOutputForward(10);
        //!leftMaster.configPeakOutputReverse(10);
    }
    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void setDrivePercent(double leftOutput, double rightOutput)
    {
        leftMaster.set(ControlMode.PercentOutput, leftOutput);
        rightMaster.set(ControlMode.PercentOutput, rightOutput);
        leftMinion.set(ControlMode.PercentOutput, leftOutput);
        rightMinion.set(ControlMode.PercentOutput, rightOutput);
    }

    public void updateDrive()
    {
        rightMaster.setInverted(true);
        rightMinion.setInverted(true);
        //see shit and stuff
        SmartDashboard.putNumber("busVoltage", leftMaster.getBusVoltage());
        SmartDashboard.putNumber("output%", leftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("outputVoltage", leftMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("sensorPos", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("sensorVelocity", leftMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("statorCurrent", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("supplyCurrent", leftMaster.getSupplyCurrent());
        SmartDashboard.putNumber("temp", leftMaster.getTemperature());

        SmartDashboard.putNumber("rbusVoltage", rightMaster.getBusVoltage());
        SmartDashboard.putNumber("routput%", rightMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("routputVoltage", rightMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("rsensorPos", rightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("rsensorVelocity", rightMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rstatorCurrent", rightMaster.getStatorCurrent());
        SmartDashboard.putNumber("rsupplyCurrent", rightMaster.getSupplyCurrent());
        SmartDashboard.putNumber("rtemp", rightMaster.getTemperature());

        SmartDashboard.putNumber("averageInchDist", averageInchesDistance);
        SmartDashboard.putNumber("averageInchVel", averageInchesPerSec);
        SmartDashboard.putNumber("distance to target", (distance - averageInchesDistance));

        leftPosNative = leftMaster.getSelectedSensorPosition();
        rightPosNative = rightMaster.getSelectedSensorPosition();
        averageNativeDistance = (leftPosNative + rightPosNative) / 2.0;

        leftPosInch = MkUtil.nativeToInches(leftPosNative);
        rightPosInch = MkUtil.nativeToInches(rightPosNative);
        averageInchesDistance = (leftPosInch + rightPosInch) / 2.0;

        leftVelNative = leftMaster.getSelectedSensorVelocity();
        rightVelNative = rightMaster.getSelectedSensorVelocity();
        averageNativePer100ms = (leftVelNative + rightVelNative) / 2.0;

        leftVelInch = MkUtil.nativePer100MstoInchesPerSec(leftVelNative);
        rightVelInch = MkUtil.nativePer100MstoInchesPerSec(rightVelNative);
        averageInchesPerSec = (leftVelInch + rightVelInch) / 2.0;
    }

    public void setMagic(double magical)
    {
        leftMaster.configMotionCruiseVelocity(1095);
        leftMaster.configMotionAcceleration(674);
        rightMaster.configMotionCruiseVelocity(1095);
        rightMaster.configMotionAcceleration(674);
        distance = magical;
        resetStuff();
    }
    public void motionMagical()
    {
        leftMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        rightMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
    }

    public boolean isMotionDone()
    {
        return Math.abs(distance - averageInchesDistance) < 0.5 && averageInchesPerSec < 0.1;
    }

    public void resetStuff()
    {
        navX.zeroYaw();
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }
    
    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
    
}

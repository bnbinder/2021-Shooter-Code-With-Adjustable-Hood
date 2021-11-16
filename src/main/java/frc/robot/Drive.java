// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import javax.management.remote.rmi.RMIIIOPServerImpl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVE;

/** Add your docs here. */
public class Drive {
    TalonFX leftMaster = new TalonFX(DRIVE.leftMasterCANID);
    TalonFX rightMaster = new TalonFX(DRIVE.rightMasterCANID);
    TalonFX leftMinion = new TalonFX(DRIVE.leftMinionCANID);
    TalonFX rightMinion = new TalonFX(DRIVE.rightMinionCANID);
    AHRS navX = new AHRS();
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navX.getRotation2d());
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

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftMinion.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMinion.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
        leftMinion.follow(leftMaster);
        rightMinion.follow(rightMaster);

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

        
        leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftMaster.configVelocityMeasurementWindow(32);

        leftMinion.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftMinion.configVelocityMeasurementWindow(32);

        rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightMaster.configVelocityMeasurementWindow(32);

        rightMinion.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightMinion.configVelocityMeasurementWindow(32);



        leftMaster.enableVoltageCompensation(true);
        leftMaster.configVoltageCompSaturation(12);

        leftMinion.enableVoltageCompensation(true);
        leftMinion.configVoltageCompSaturation(12);

        rightMaster.enableVoltageCompensation(true);
        rightMaster.configVoltageCompSaturation(12);

        rightMinion.enableVoltageCompensation(true);
        rightMinion.configVoltageCompSaturation(12);



        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        leftMinion.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        leftMinion.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        rightMinion.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        rightMinion.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);



        leftMaster.configPeakOutputForward(1);
        leftMaster.configPeakOutputReverse(.80);

        leftMinion.configPeakOutputForward(1);
        leftMinion.configPeakOutputReverse(.80);

        rightMaster.configPeakOutputForward(1);
        rightMaster.configPeakOutputReverse(.80);

        rightMinion.configPeakOutputForward(1);
        rightMinion.configPeakOutputReverse(.80);



        leftMaster.configClosedLoopPeakOutput(0, 1);
        leftMinion.configClosedLoopPeakOutput(0, 1);
        rightMaster.configClosedLoopPeakOutput(0, 1);
        rightMinion.configClosedLoopPeakOutput(0, 1);



        leftMaster.configOpenloopRamp(DRIVE.driveOpenRampRate);
        leftMinion.configOpenloopRamp(DRIVE.driveOpenRampRate);
        rightMaster.configOpenloopRamp(DRIVE.driveOpenRampRate);
        rightMinion.configOpenloopRamp(DRIVE.driveOpenRampRate);



        leftMaster.configClosedloopRamp(DRIVE.driveCloseRampRate);
        leftMinion.configClosedloopRamp(DRIVE.driveCloseRampRate);
        rightMaster.configClosedloopRamp(DRIVE.driveCloseRampRate);
        rightMinion.configClosedloopRamp(DRIVE.driveCloseRampRate);


        leftMaster.configMotionSCurveStrength(6);
        leftMinion.configMotionSCurveStrength(6);
        rightMaster.configMotionSCurveStrength(6);
        rightMinion.configMotionSCurveStrength(6);
        //swerd code said 6, so 6 it is


        leftMaster.configAllowableClosedloopError(0, 1);
        leftMinion.configAllowableClosedloopError(0, 1);
        rightMaster.configAllowableClosedloopError(0, 1);
        rightMinion.configAllowableClosedloopError(0, 1);
        //this too, since sensor units are super big, so 1 is small i guess
    
    
        leftMaster.configNeutralDeadband(0.001); 
        leftMinion.configNeutralDeadband(0.001); 
        rightMaster.configNeutralDeadband(0.001); 
        rightMinion.configNeutralDeadband(0.001); 
        //these are useless, but since in the swerd code, he is god so he knows best




        //gets velocity in past 10ms when determining what velocity to have (motion magic)
        //?     test and add later
        //TODO leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

        //how many velocities to compare/average?
        //?     test and add later
        //TODO leftMaster.configVelocityMeasurementWindow(32);

        //enable and set the max amount of voltage a motor can use?
        //?     research and add later
        //!leftMaster.enableVoltageCompensation(true);
        //!leftMaster.configVoltageCompSaturation(10);

        //if motor takes longer than 20ms to get from start to finish, can bus not utilizing enough and stuff wrong?
        //?     research and add later
        //!leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        //!leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        //peak output 10% forward and back?
        //?     research and test
        //!leftMaster.configPeakOutputForward(10);
        //!leftMaster.configPeakOutputReverse(10);

        //sets time (ms) of how long one loop in closed loop is?
        //?     research and add later
        //!leftMaster.configClosedLoopPeriod(0,1);

        //what this do?
        //?     test and add later
        //!leftMaster.configAllowableClosedloopError(0, 1);

        //makes sure motors dont go above 1 (100%)?
        //?     test and add later
        //!leftMaster.configClosedLoopPeakOutput(0, 1);

        //when input is below deadband but above/below zero, motors dont move
        //see documentation
        //?     test and add later
        //!leftMaster.configNeutralDeadband(0.01);

        //what this do?
        //motor not go below this number when forward?
        //?     test and add later
        //!leftMaster.configNominalOutputForward(0);

        //what this do?
        //motor not go below this number when backward?
        //?     test and add later
        //!leftMaster.configNominalOutputReverse(0);

        //during open loop, 0.5 seconds before motor go from 0 to selected output
        //?     test and add later
        //!leftMaster.configOpenloopRamp(DRIVE.driveOpenRampRate);

        //during close loop, 0.5 seconds before motor go from 0 to selected output
        //?     test and add later
        //!leftMaster.configClosedloopRamp(DRIVE.driveCloseRampRate);

        //sets pid fI to set number, but what is pidIdx?
        //?     research
        //!leftMaster.setIntegralAccumulator(iaccum, pidIdx, timeoutMs));
        //!leftMaster.setIntegralAccumulator(iaccum);

        //limits motor output (how much motor uses)?
        //?     research
        //!leftMaster.configStatorCurrentLimit();

        //limits motor input (how much motor takes)?
        //?     research
        //!leftMaster.configSupplyCurrentLimit();
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

        SmartDashboard.putNumber("odometryY", MkUtil.nativePer100MstoInchesPerSec(odometry.getPoseMeters().getY()));
        SmartDashboard.putNumber("odometryX", MkUtil.nativePer100MstoInchesPerSec(odometry.getPoseMeters().getY()));
        SmartDashboard.putNumber("odometrydeg", odometry.getPoseMeters().getRotation().getDegrees());

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

    
        odometry.update(navX.getRotation2d(), leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition());
    }

    public void resetOdo()
    {
        resetStuff();
        odometry.resetPosition(new Pose2d(), navX.getRotation2d());
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

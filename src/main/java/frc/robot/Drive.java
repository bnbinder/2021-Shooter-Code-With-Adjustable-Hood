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
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DRIVE;

/** Add your docs here. */
public class Drive {
    TalonFX leftMaster = new TalonFX(DRIVE.leftMasterCANID);
    TalonFX rightMaster = new TalonFX(DRIVE.rightMasterCANID);
    TalonFX leftMinion = new TalonFX(DRIVE.leftMinionCANID);
    TalonFX rightMinion = new TalonFX(DRIVE.rightMinionCANID);
    private Drive()
    {
        //sets factory defualt, good?
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightMinion.configFactoryDefault();
        leftMinion.configFactoryDefault();
    
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftMinion.setNeutralMode(NeutralMode.Brake);
        rightMinion.setNeutralMode(NeutralMode.Brake);

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
        //!leftMaster.configOpenloopRamp(0.5);

        //during close loop, 0.5 seconds before motor go from 0 to selected output
        //?     test later
        //!leftMaster.configClosedloopRamp(0.5);

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
    }
    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateDrive()
    {
        //see shit and stuff
        SmartDashboard.putNumber("busVoltage", leftMaster.getBusVoltage());
        SmartDashboard.putNumber("output%", leftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("outputVoltage", leftMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("sensorPos", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("sensorVelocity", leftMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("statorCurrent", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("supplyCurrent", leftMaster.getSupplyCurrent());
        SmartDashboard.putNumber("temp", leftMaster.getTemperature());
    }

    public void motionMagical()
    {
        leftMaster.set(ControlMode.MotionMagic);
    }
    
    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
    
}

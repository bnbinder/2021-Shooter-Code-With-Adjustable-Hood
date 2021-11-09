// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
    
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftMinion.setNeutralMode(NeutralMode.Brake);
        rightMinion.setNeutralMode(NeutralMode.Brake);

        //what this do?
        // ! leftMaster.configAllowableClosedloopError(0, 1);

        //makes sure motors dont go above 1 (100%)?
        leftMaster.configClosedLoopPeakOutput(0, 1);

        //when input is below deadband but above/below zero, motors dont move
        //see documentation
        leftMaster.configNeutralDeadband(0.01);

        //what this do?
        //motor not go below this number when forward?
        leftMaster.configNominalOutputForward(0);

        //what this do?
        //motor not go below this number when backward?
        leftMaster.configNominalOutputReverse(0);

        //during open loop, 0.5 seconds before motor go from 0 to selected output
        leftMaster.configOpenloopRamp(0.5);

        //during close loop, 0.5 seconds before motor go from 0 to selected output
        leftMaster.configClosedloopRamp(0.5);

        //
    }
}

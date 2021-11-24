// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HOOD;
import frc.robot.Constants.SHOOT;
import frc.robot.Drive;
import frc.robot.Constants.VISION;

import javax.swing.event.MenuDragMouseEvent;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.util.ArrayBuilders.ShortBuilder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** Add your docs here. */
public class Shooter {
    TalonFX ShootRight = new TalonFX(SHOOT.shootRightCANID);
    TalonFX ShootLeft= new TalonFX(SHOOT.shootLeftCANID);
    TalonFX ShootHood = new TalonFX(HOOD.shootHoodCANID);

    private Drive mDrive = Drive.getInstance();
    //private Shooter mShoot = Shooter.getInstance(); im a moron

    private double leftShootPosNative, rightShootPosNative, 
                   leftShootPosInch, rightShootPosInch, 
                   leftShootVelInch, rightShootVelInch, 
                   leftShootVelNative, rightShootVelNative, 
                   averageShootNativeDistance, averageShootInchesDistance, 
                   averageShootNativePer100ms, averageShootInchesPerSec;

    private double ff;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(HOOD.hoodMaxVel, HOOD.hoodMaxAccel);
    private ProfiledPIDController hoodPID = new ProfiledPIDController(HOOD.hoodKP, HOOD.hoodKI, HOOD.hoodKD, constraints);


    private Shooter()
    {
            //TODO have two pids one for hood and one for shoot
        ShootLeft.configFactoryDefault();
        ShootRight.configFactoryDefault();
        ShootHood.configFactoryDefault();
        
        ShootLeft.follow(ShootRight);

        ShootLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        ShootRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        ShootHood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        ShootRight.config_kP(0, SHOOT.kP);
        ShootRight.config_kI(0, SHOOT.kI);
        ShootRight.config_kD(0, SHOOT.kD);
        ShootRight.config_kF(0, SHOOT.kF);

        ShootLeft.config_kP(0, SHOOT.kP);
        ShootLeft.config_kI(0, SHOOT.kI);
        ShootLeft.config_kD(0, SHOOT.kD);
        ShootLeft.config_kF(0, SHOOT.kF);

        ShootHood.setNeutralMode(NeutralMode.Brake);
        ShootRight.setNeutralMode(NeutralMode.Coast);
        ShootLeft.setNeutralMode(NeutralMode.Coast);
                
        ShootLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        ShootLeft.configVelocityMeasurementWindow(32);

        ShootRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        ShootRight.configVelocityMeasurementWindow(32);

        ShootHood.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        ShootHood.configVelocityMeasurementWindow(32);


        ShootLeft.enableVoltageCompensation(true);
        ShootLeft.configVoltageCompSaturation(12);

        ShootRight.enableVoltageCompensation(true);
        ShootRight.configVoltageCompSaturation(12);

        ShootHood.enableVoltageCompensation(true);
        ShootHood.configVoltageCompSaturation(12);




        ShootLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        ShootLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        ShootRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        ShootRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        ShootHood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        ShootHood.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);



/*        ShootLeft.configPeakOutputForward(1);
        ShootLeft.configPeakOutputReverse(1);

        ShootRight.configPeakOutputForward(1);
        ShootRight.configPeakOutputReverse(1);
*/
        //ShootHood.configPeakOutputForward(.3);
        //ShootHood.configPeakOutputReverse(-.3);
        //!fucked with the motors, since we arent using built in pidf controller



        ShootLeft.configClosedLoopPeakOutput(0, 1);
        ShootRight.configClosedLoopPeakOutput(0, 1);
        //ShootHood.configClosedLoopPeakOutput(0, 0.05);



        ShootLeft.configOpenloopRamp(SHOOT.shootOpenRampRate);
        ShootRight.configOpenloopRamp(SHOOT.shootOpenRampRate);
        ShootHood.configOpenloopRamp(HOOD.shootHoodOpenRamp);


        ShootLeft.configClosedloopRamp(SHOOT.shootCloseRampRate);
        ShootRight.configClosedloopRamp(SHOOT.shootCloseRampRate);
        //ShootHood.configClosedloopRamp(HOOD.shootHoodCloseRamp);

        ShootLeft.configMotionSCurveStrength(1);
        ShootRight.configMotionSCurveStrength(1);
        //ShootHood.configMotionSCurveStrength(1);
        //swerd code said 6, so 6 it is


        ShootLeft.configAllowableClosedloopError(0, 1);
        ShootRight.configAllowableClosedloopError(0, 1);
        ShootHood.configAllowableClosedloopError(0, 1);
        //this too, since sensor units are super big, so 1 is small i guess
    
    
        ShootLeft.configNeutralDeadband(0.001); 
        ShootRight.configNeutralDeadband(0.001); 
        ShootHood.configNeutralDeadband(0.001); 
        //this too

        /*
        ShootHood.config_kP(0, SHOOT.hoodKP);
        ShootHood.config_kI(0, SHOOT.hoodKI);
        ShootHood.config_kD(0, SHOOT.hoodKD);
        ShootHood.config_kF(0, SHOOT.hoodKF);
        */
        //TODO hope commenting this shit doesnt affect pid controller
        //! commenting it out since it doesnt affect pid controller, but the built in pidf (i think and hope)
    }
    
    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateShoot()
    {
        SmartDashboard.putNumber("leftshootsensorV", ShootLeft.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightshootsensorV", ShootRight.getSelectedSensorVelocity());
        SmartDashboard.putNumber("hoodsensor", ShootHood.getSelectedSensorPosition());

        //SmartDashboard.putNumber("gvcgyvcgvucvug", ShootHood.getSelectedSensorVelocity());
        ShootRight.setInverted(true);

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

       // SmartDashboard.putNumber("bobs burgers bobs burgers", shootCalculateShit(4000));
        
    }

    public void shootPercent(double shoot)
    {
        ShootLeft.set(ControlMode.PercentOutput, shoot);
        ShootRight.set(ControlMode.PercentOutput, shoot);
    }

    public void shootHoodPercent(double shoot)
    {
        ShootHood.set(ControlMode.PercentOutput, shoot);
    }

    public void shootVelocity(double vel)
    {
        ShootLeft.set(ControlMode.Velocity, vel);
        ShootRight.set(ControlMode.Velocity, vel);
    }

    public void ShootHoodPosition(double pos)
    {
        pos = MkUtil.limit(pos, 100, 6400);
        //ShootHood.set(ControlMode.Position, pos);

        //!    mfw no position control :(
    }

    public void setHoodPos(double pos)
    {
        ShootHood.setSelectedSensorPosition(pos);
    }
    


    public void resetKI()
    {
        hoodPID.reset(ShootHood.getSelectedSensorPosition());
    }



    public double getCameraHeight()
    {
        //TODO find greer ratio and hope this works
        return VISION.hoodLength * Math.sin(getCameraAngle());
        //sin(theta) = opp/hyp
        //hypsintheta = opp
    }

    public double getCameraAngle()
    {
        return VISION.angleNativeCameraAtRest + MkUtil.nativeToDegrees(ShootHood.getSelectedSensorPosition(), VISION.totalGreerRatio);
    }

    public double getCamerAngleWithout()
    {
        return MkUtil.nativeToDegrees(ShootHood.getSelectedSensorPosition(), VISION.totalGreerRatio);
    }

    public double ffHood(double setpoint)
    {
        //cos zero = pi/2
        //set zero to minimum steady state
        return VISION.setPoint0 * (Math.cos(((Constants.kPi * 1.1) / (VISION.maxRotate * 2)) * setpoint));
    }

    public double ffshoot(double setpoint)
    {
        return (((SHOOT.maxShootError * setpoint) / SHOOT.maxNativeVelocity));
    }

    public double geterror()
    {
        return hoodPID.getPositionError();
    }

    public double getveleror()
    {
        return hoodPID.getVelocityError();
    }



    public double getHoodSensorPos()
    {
        return ShootHood.getSelectedSensorPosition();
    }

   
   
   
   
    public void resetPID()
    {
        hoodPID.reset(0, 0);
        hoodPID.setGoal(0);
    }


    public double shootCalculateShit(double goal)
    {
        goal = MkUtil.limit(goal, 100, 6400);
        ff = ffHood(goal);
        goal = MkUtil.limit(goal + ff, 100, 6400);
        //i dont know if this is redundant but idk if ff (if ff is added to first limit) will be
        //goal that is not within limits of 100 and 6400
        //if goal is 7000, and ff is in the limit, idk if ff will equal ff of 7000 or ff of 6400
        return hoodPID.calculate(getHoodSensorPos(), goal);
    }
    
    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    }
}

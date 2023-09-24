// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.persistent.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    // Timer timer;
    Command auton;
    // ADIS16470_IMU gyro;

    private final RobotContainer robotContainer = new RobotContainer();

    public Robot() {
        // this.gyro = new ADIS16470_IMU();
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer.setSwerveDriveCurrentLimitTurn(Constants.Drivetrain.kTurnCurrentLimit);
        robotContainer.setSwerveDriveCurrentLimitDrive(Constants.Drivetrain.kDriveCurrentLimit);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.clearStickyFaults();
        robotContainer.outputPitch();
        
        // SmartDashboard.put robotContainer.getCameraFeed();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        Constants.isEnabled = false;

        CommandScheduler.getInstance().cancelAll();
        robotContainer.setTurnDefaultMode(NeutralMode.Coast);
        robotContainer.setDriveDefaultMode(NeutralMode.Coast);
    }

    @Override
    public void disabledPeriodic() {
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        Constants.isEnabled = false;
        Constants.isEnabled = false;
        robotContainer.setTurnDefaultMode(NeutralMode.Brake);
        robotContainer.setDriveDefaultMode(NeutralMode.Brake);
        robotContainer.autonInit();
        this.auton = robotContainer.getAutonomousCommand();

        if(auton!=null) {
            auton.schedule();
        } else {
            System.out.println("AUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\n");
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once each time the robot enters teleoperated mode. */ 
    @Override
    public void teleopInit() {
        Constants.isEnabled = true;
        Constants.isEnabled = true;
        CommandScheduler.getInstance().cancelAll();
        robotContainer.setTurnDefaultMode(NeutralMode.Coast);
        robotContainer.setDriveDefaultMode(NeutralMode.Coast);
        robotContainer.configureButtonBindings();
        robotContainer.getSwerveDrive().zeroDrive();
        
    }
    
    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() {
        robotContainer.initRumbleTimer();

        // robotContainer.outputAutonLog();
        // robotContainer.outputAutonLog();
    }



}
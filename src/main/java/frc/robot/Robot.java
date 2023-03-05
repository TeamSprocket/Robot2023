// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    Timer timer;
    Command auton;

    private final RobotContainer robotContainer = new RobotContainer();

    public Robot() {
        
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
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
        // SmartDashboard.putNumber("Robot Wheel Position Meters", robotContainer.getSwerveDrive().getDrivePosition());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        robotContainer.autonInit();

        this.auton = robotContainer.getAutonomousCommand();
        // new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.25, 0.0, new Rotation2d(0.0)), 1);
        // timer.start();

        if(auton!=null) {
            auton.schedule();
        } else {
            System.out.println("AUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\nAUTON IS NULL\n");
        }

        
        
        // timer = new Timer();
        // timer.start();
        
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

        // double time = timer.get();
        // if (time < 1) {
        //     new SwerveDriveCmd(swerveDrive, 0.0, 0.0, 0.0));
        // }
 
    }

    /** This function is called once each time the robot enters teleoperated mode. */ 
    @Override
    public void teleopInit() {
        System.out.println("TELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\n");
        robotContainer.configureButtonBindings();
        CommandScheduler.getInstance().cancelAll();
    }
    
    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
}
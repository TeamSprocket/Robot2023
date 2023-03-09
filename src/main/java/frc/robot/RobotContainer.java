package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.commands.auton.BalanceOnChargeStationVertical;
import frc.robot.commands.auton.OneMeterForward;
// import frc.robot.commands.auton.SwerveAutonTest;
import frc.robot.commands.auton.WaitTimed;
import frc.robot.commands.instant.ToggleClaw;
import frc.robot.commands.instant.ToggleCompressor;
import frc.robot.commands.macro.ElevatePosition;
import frc.robot.commands.macro.MoveArmPosition;
import frc.robot.commands.macro.MoveWristAngle;
import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.SetHome;
import frc.robot.commands.macro.SetLow;
import frc.robot.commands.macro.SetMid;
import frc.robot.commands.macro.ShootClaw;
import frc.robot.commands.macro.SwerveDriveCmdPrecise;
import frc.robot.commands.macro.ToggleSwervePrecise;
import frc.robot.commands.macro.timed.DeportArm;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetHumanPlayerTimed;
import frc.robot.commands.macro.timed.SetLowTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.SetHumanPlayer;
import frc.robot.commands.persistent.Elevate;
import frc.robot.commands.persistent.MoveArmJoystick;
import frc.robot.commands.persistent.MoveWristManual;
import frc.robot.commands.persistent.RollClaw;
// import frc.robot.commands.auton.SwerveAutonTest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PCH;
import frc.robot.subsystems.PDH;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;


// importing stuff for path planner
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.lang.Object;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.PrintCommand;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public final class RobotContainer {
	// Controllers
	private final XboxController driver = new XboxController(0);
	private final XboxController operator = new XboxController(1);

	// Smartdashboard
	// Subsystems
	private final PCH pch = new PCH();
	private final PDH pdh = new PDH();
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	private final Wrist wrist = new Wrist();
	private final Claw claw = new Claw();

	// Manual Autons
	SendableChooser<Command> chooser = new SendableChooser<>();
	// Command swerveAutonTest = new SwerveAutonTest(swerveDrive);

	public RobotContainer() {
		// chooser.setDefaultOption("Swerve Auton Test", swerveAutonTest);

		SmartDashboard.putData(chooser);
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
	 * then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	public void configureButtonBindings() {
		swerveDrive.setDefaultCommand(new SwerveDriveCmd(
				swerveDrive,
				// X
				() -> -driver.getLeftY(),
				// Y
				() -> driver.getLeftX(),
				// T
				() -> -driver.getRightX()));

		// Elevator
		// TODO CHECK THE POSITIONS OF THE ELEVATOR
		// elevator.setDefaultCommand(new Elevate(elevator,
		// (operator.getLeftTriggerAxis() - operator.getRightTriggerAxis())));
		elevator.setDefaultCommand(new Elevate(elevator, operator));
		arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
		wrist.setDefaultCommand(new MoveWristManual(wrist, operator));
		claw.setDefaultCommand(new RollClaw(claw, driver));

		// Swerve Drive (instant command reset heading)
		new JoystickButton(driver,
				RobotMap.Controller.RESET_GYRO_HEADING_BUTTON_ID).whenPressed(() -> swerveDrive.zeroHeading());
		// new JoystickButton(driver, 2).whenPressed(() -> swerveDrive.zeroTalons());
		new JoystickButton(driver, 4).whenPressed(new ToggleCompressor(pch, driver));
		new JoystickButton(driver, 5).whenPressed(new ToggleClaw(claw));
		// new JoystickButton(driver, 3).whenHeld(new ShootClaw(10));

		// new JoystickButton(driver, 7).whenPressed(() -> swerveDrive.zeroTalons());

		new JoystickButton(operator, 1).whenHeld(new SetLow(elevator, arm, wrist));
		new JoystickButton(operator, 2).whenHeld(new SetHumanPlayer(elevator, arm, wrist));
		new JoystickButton(operator, 3).whenHeld(new SetHome(elevator, arm, wrist));
		new JoystickButton(operator, 4).whenHeld(new SetHigh(elevator, arm, wrist));
		new JoystickButton(operator, 5).whenHeld(new SetMid(elevator, arm, wrist));
		new JoystickButton(operator, 6).whenHeld(new DeportArm(elevator, arm, wrist));

		// new POVButton(driver, 90).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 1,
		// 0));
		// new POVButton(driver, 270).whenHeld(new SwerveDriveCmdPrecise(swerveDrive,
		// -1, 0));
		// new POVButton(driver, 0).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 0,
		// 1));
		// new POVButton(driver, 180).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 0,
		// -1));

		// new POVButton(driver, 45).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 1,
		// 1));
		// new POVButton(driver, 135).whenHeld(new SwerveDriveCmdPrecise(swerveDrive,
		// -1, 1));
		// new POVButton(driver, 225).whenHeld(new SwerveDriveCmdPrecise(swerveDrive,
		// -1, -1));
		// new POVButton(driver, 315).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 1,
		// -1));

		// new JoystickButton(operator, 5).whenPressed(new SetElevatorBase(elevator));

	}

	public void autonInit() {
		swerveDrive.zeroTalons();
		swerveDrive.zeroHeading();
		pdh.clearStickyFaults();

	}

	public void clearStickyFaults() {
		pdh.clearStickyFaults();
		pch.clearStickyFaults();
	}

	// AUTON
	// public Command getAutonomousCommand() {
	public Command getAutonomousCommand() {


//	autopath
		// List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3)); // maxVelocity:4 and maxAcceleration:3

		// HashMap<String, Command> eventMap = new HashMap<>();
		// eventMap.put("marker1", new PrintCommand("Passed marker 1"));


		// SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
		// 	// swerveDrive.getDrivePosition(), 
		// 	null,
		// 	null, 
		// 	null, 
		// 	null, 
		// 	null, 
		// 	eventMap, 
		// 	null);

		// return (Command) (new SequentialCommandGroup(
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.2, new
		// Rotation2d(0.0)), 1),
		// new WaitTimed(3),
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new
		// Rotation2d(0.1)), 1)
		// ));

		// Do nothing (0)
		// return (Command) (new SequentialCommandGroup(
		// ));

		// Move back (4)
		// return (Command) (new SequentialCommandGroup(
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.2, new
		// Rotation2d(0.1)), 4) //4
		// ));

		// Place cone & move back (9.5)
		// return (Command) (new SequentialCommandGroup(
		// new DeportArm(elevator, arm, wrist), //2
		// new SetHighTimed(elevator, arm, wrist, 2), //2
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.2, new
		// Rotation2d(0.0)), 1), //1
		// new ToggleClaw(claw), //0
		// new WaitTimed(0.5), //0.5
		// new ParallelCommandGroup( //4
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.2, new
		// Rotation2d(0.0)), 4), //4*
		// new SetHomeTimed(elevator, arm, wrist, 4) //4*
		// )
		// ));

		// Place cube & move back (10)
		return (Command) (new SequentialCommandGroup(
				new DeportArm(elevator, arm, wrist), // 2
				new SetHighTimed(elevator, arm, wrist, 2), // 2
				new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.25, new Rotation2d(0.0)), 2), // 1
				// new WaitTimed(0.5),
				new RollClawTimed(claw, 0.5, 1), // 1
				new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 1),
				new ParallelCommandGroup(
						new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 3.25),
						new SetHomeTimed(elevator, arm, wrist, 4) // 4*
				)));

		// Place Cube Only
		// return (Command) (new SequentialCommandGroup(
		// new DeportArm(elevator, arm, wrist), //2
		// new SetHighTimed(elevator, arm, wrist, 2), //2
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.25, new
		// Rotation2d(0.0)), 2), //1
		// // new WaitTimed(0.5),
		// new RollClawTimed(claw, 0.5, 1) //1
		// // new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.25, new
		// Rotation2d(0.0)), 2),
		// // new ParallelCommandGroup(
		// // new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new
		// Rotation2d(0.0)), 3.25),
		// // new SetHomeTimed(elevator, arm, wrist, 4) //4*
		// // )
		// ));

		// MOVE FORWARD ONLY
		// return (Command) (new SequentialCommandGroup(
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new
		// Rotation2d(0.0)), 2)
		// ));

		// Balance Charging Station BLUE (idk)
		// return (Command) (new SequentialCommandGroup(
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new
		// Rotation2d(0.0)), 2),
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.24, 0.0, new
		// Rotation2d(0.0)), 2),
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new
		// Rotation2d(0.0)), 1.75), //4
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new
		// Rotation2d(0.01), 0.5))
		// ));pz

		// Balance Charging Station RED (idk)
		// return (Command) (new SequentialCommandGroup(
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new
		// Rotation2d(0.0)), 2),
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.24, 0.0, new
		// Rotation2d(0.0)), 2),
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new
		// Rotation2d(0.0)), 1.75), //4
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new
		// Rotation2d(0.01), 0.5))
		// ));

		// PID Charging Station (idkkkkkkkk)
		// return (Command) (new SequentialCommandGroup(
		// new OneMeterForward(swerveDrive)
		// ));

		//////////////////////////////////////////

		// Command autonRoutine = new SequentialCommandGroup(
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, 0.2, new Rotation2d(0.0)),
		// 1),
		// new WaitTimed(3),
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, -0.2, new
		// Rotation2d(0.0)), 1)
		// );
		// return autonRoutine;

		// new DeportArm(elevator, arm, wrist),
		// new SetHighTimed(elevator, arm, wrist, 3),
		// new SetHumanPlayerTimed(elevator, arm, wrist, 2),
		// new RollClawTimed(claw, 1, 1),
		// new ParallelCommandGroup(
		// new SetLowTimed(elevator, arm, wrist, 5),
		// new RollClawTimed(claw, -0.5, 5)
		// ),
		// new SetHighTimed(elevator, arm, wrist, 3),
		// new RollClawTimed(claw, 1, 1),
		// new SetHomeTimed(elevator, arm, wrist, 3)

		// return chooser.getSelected();
	}

	// // Create Trajectory Speed/Settings
	// TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
	// Constants.Drivetrain.kMaxSpeedMetersPerSecond,
	// Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond)
	// .setKinematics(Constants.Drivetrain.driveKinematics);

	// // Create auton trajectory
	// Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
	// // Initial point/rotation
	// new Pose2d(0, 0, new Rotation2d(0)),
	// // Points to traverse to
	// List.of(
	// new Translation2d(1, 0),
	// new Translation2d(1, -1)
	// ),
	// // Final point + rotation
	// new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
	// trajectoryConfig
	// );
	// // return trajectory;
	// // }

	// // PID Controller for tracking trajectory (profiled = limits max speed/rot)
	// // PIDController xController = new
	// PIDController(Constants.Drivetrain.PID_CONTROLLER_X_P, 0, 0);
	// // PIDController yController = new
	// PIDController(Constants.Drivetrain.PID_CONTROLLER_Y_P, 0, 0);
	// // ProfiledPIDController tController = new
	// ProfiledPIDController(Constants.Drivetrain.PID_CONTROLLER_T_P, 0, 0,
	// // new TrapezoidProfile.Constraints(
	// // Constants.Drivetrain.kPhysicalMaxAngularSpeedRadiansPerSecond,
	// // Constants.Drivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
	// // tController.enableContinuousInput(-Math.PI, Math.PI);

	// Follow trajectory command
	// SwerveControllerCommand swerveControllerCommand = new
	// SwerveControllerCommand(
	// trajectory,
	// swerveDrive::getPose,
	// Constants.Drivetrain.driveKinematics,
	// xController,
	// yController,
	// tController,
	// swerveDrive::setModuleStates,
	// swerveDrive
	// );

	// Inits, wrapup, returns
	// return new SequentialCommandGroup(
	// new InstantCommand(() ->
	// swerveDrive.resetOdometer(trajectory.getInitialPose())),
	// swerveControllerCommand,
	// new InstantCommand(() -> swerveDrive.stopModules())
	// );

	// } @Override
}
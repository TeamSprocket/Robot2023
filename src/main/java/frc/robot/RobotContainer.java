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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunMotor;
import frc.robot.commands.SwerveDriveCmd;
// import frc.robot.commands.auton.OneBall;
// import frc.robot.commands.auton.ZeroBall;
// import frc.robot.commands.auton.ThreeBall;
// import frc.robot.commands.auton.TwoBallLeft;
// import frc.robot.commands.auton.TwoBallRight;
// import frc.robot.commands.instant.ActuateClimb;
// import frc.robot.commands.instant.ToggleCompressor;
// import frc.robot.commands.instant.ToggleIntake;
// import frc.robot.commands.macro.Shoot;
// import frc.robot.commands.persistent.BlingBling;
// import frc.robot.commands.persistent.ClimbArmManual;
// import frc.robot.commands.persistent.Drive;
// import frc.robot.commands.persistent.FeedManual;
// import frc.robot.commands.persistent.RollIntakeManual;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Feeder;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.LEDStrip;
// import frc.robot.subsystems.PCH;
// import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public final class RobotContainer {
	

	// private final Drivetrain drivetrain = new Drivetrain();
	// private final PCH pch = new PCH();
	// private final Climber climber = new Climber();
	// private final Intake intake = new Intake();
	// private final Shooter shooter = new Shooter();
	// private final Feeder feeder = new Feeder();
	// private final LEDStrip ledStrip = new LEDStrip();

	// private final Joystick leftJoystick = new Joystick(0);
	// private final Joystick rightJoystick = new Joystick(1);

	private final XboxController driveController = new XboxController(0);
	private final XboxController gamepad = new XboxController(1);
	// private final RunMotor runMotor = new RunMotor(driveController, 23);

	// private final Command oneBall = new OneBall(drivetrain, feeder, intake, shooter, pch);
	// private final Command twoBallRight = new TwoBallRight(drivetrain, intake, feeder, shooter);
	// private final Command twoBallLeft = new TwoBallLeft(drivetrain, intake, feeder, shooter);
	// private final Command zeroBall = new ZeroBall(drivetrain);
	// private final Command threeBall = new ThreeBall(drivetrain, intake, feeder, shooter);
	SendableChooser<Command> chooser = new SendableChooser<>(); 

	// Swerve Drive
	private final SwerveDrive swerveDrive = new SwerveDrive();


	public RobotContainer() {
		// Swerve Drive
		
		// WPI_TalonFX talonTest = new WPI_TalonFX(23);
		
		

		
		configureButtonBindings();

		// chooser.addOption("1-ball", oneBall);
		// chooser.addOption("0-ball", zeroBall);
		// chooser.addOption("2-ball, human player", twoBallRight);
		// chooser.setDefaultOption("2-ball, NOT human player", twoBallLeft);
		// chooser.addOption("BIG BALLS", threeBall);
		SmartDashboard.putData(chooser);
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
	 * then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		swerveDrive.setDefaultCommand(new SwerveDriveCmd(
			swerveDrive, 
			// X
			() -> driveController.getLeftX(), 
			// Y
			() -> driveController.getLeftY(), 
			// T
			() -> driveController.getRightX(), 
			() -> Constants.Drivetrain.IS_FIELD_ORIENTED));
		// new JoystickButton(driveController, 1).whenPressed(swerveDrive.resetGyro());

		// drivetrain.setDefaultCommand(new Drive(drivetrain, leftJoystick, rightJoystick));
		// new JoystickButton(gamepad, 4).whenPressed(new ToggleCompressor(pch, gamepad));
		// new JoystickButton(gamepad, 6).whenPressed(new ActuateClimb(climber, true));
		// new JoystickButton(gamepad, 5).whenPressed(new ActuateClimb(climber, false));
		// new JoystickButton(gamepad, 3).whenPressed(new ToggleIntake(intake));
		// new JoystickButton(gamepad, 2).whenHeld(new Shoot(shooter));
		// // shooter.setDefaultCommand(new RollShooterManual(shooter, gamepad));
		// feeder.setDefaultCommand(new FeedManual(feeder, gamepad));
		// intake.setDefaultCommand(new RollIntakeManual(intake, gamepad));
		// ledStrip.setDefaultCommand(new BlingBling(ledStrip, shooter));
		// 	climber.setDefaultCommand(new ClimbArmManual(climber, gamepad));

		// Swerve Drive (instant command reset heading)
		new JoystickButton(driveController,
		 RobotMap.Controller.RESET_GYRO_HEADING_BUTTON_ID).whenPressed(() -> swerveDrive.zeroHeading());
	}

	// AUTON
// 	public Command getAutonomousCommand() {
// 		// return chooser.getSelected();

// 		// Create Trajectory Speed/Settings
// 		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
// 			Constants.Drivetrain.kMaxSpeedMetersPerSecond,
// 				Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond)
// 				.setKinematics(Constants.Drivetrain.driveKinematics);

// 		// Create auton trajectory
// 		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
// 			// Initial point/rotation
// 			new Pose2d(0, 0, new Rotation2d(0)),
// 			// Points to traverse to
// 			List.of(
// 				new Translation2d(1, 0),
// 				new Translation2d(1, -1)
// 			),
// 			// Final point + rotation
// 			new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
// 			trajectoryConfig
// 		);

// 		// PID Controller for tracking trajectory (profiled = limits max speed/rot)
// 		PIDController xController = new PIDController(Constants.Drivetrain.PID_CONTROLLER_X_P, 0, 0);
// 		PIDController yController = new PIDController(Constants.Drivetrain.PID_CONTROLLER_Y_P, 0, 0);
// 		ProfiledPIDController tController = new ProfiledPIDController(Constants.Drivetrain.PID_CONTROLLER_T_P, 0, 0,
// 			new TrapezoidProfile.Constraints(
// 				Constants.Drivetrain.kPhysicalMaxAngularSpeedRadiansPerSecond,
// 				Constants.Drivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
// 		tController.enableContinuousInput(-Math.PI, Math.PI);

// 		// Follow trajectory command
// 		// SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
// 		// 	trajectory,
// 		// 	swerveDrive::getPose,
// 		// 	Constants.Drivetrain.driveKinematics,
// 		// 	xController,
// 		// 	yController,
// 		// 	tController,
// 		// 	swerveDrive::setModuleStates,
// 		// 	swerveDrive
// 		// );

// 		// Inits, wrapup, returns
// 		// return new SequentialCommandGroup(
// 		// 	new InstantCommand(() -> swerveDrive.resetOdometer(trajectory.getInitialPose())), 
// 		// 	swerveControllerCommand, 
// 		// 	new InstantCommand(() -> swerveDrive.stopModules())
// 		// );

// 	}	@Override
}

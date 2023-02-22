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
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.commands.macro.ElevatePosition;
import frc.robot.commands.macro.MoveArmPosition;
// import frc.robot.commands.auton.SwerveAutonTest;
import frc.robot.commands.peresistent.ElevateJoystick;
import frc.robot.commands.peresistent.MoveArmJoystick;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public final class RobotContainer {
	//Controllers
	private final XboxController driver = new XboxController(0);
	private final XboxController operator = new XboxController(1);

	//Smartdashboard
	//Subsystems
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	private final Wrist wrist = new Wrist();

	// Manual Autons
	SendableChooser<Command> chooser = new SendableChooser<>();  
	// SwerveAutonTest swerveAutonTest = new SwerveAutonTest(swerveDrive);

	public RobotContainer() {
		// chooser.addOption("Swerve Auton Test", swerveAutonTest);

		configureButtonBindings();
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
			() -> -driver.getLeftY(), 
			// Y
			() -> driver.getLeftX(), 
			// T
			() -> -driver.getRightX()));

		// Swerve Drive (instant command reset heading)
		new JoystickButton(driver,
		 	RobotMap.Controller.RESET_GYRO_HEADING_BUTTON_ID).whenPressed(() -> swerveDrive.zeroHeading());
		new JoystickButton(driver, 2).whenPressed(() -> swerveDrive.zeroTalons());
		// new JoystickButton(driveController, 3).whenPressed(() -> swerveDrive.zeroTalonsABS());

		// Elevator
		//TODO CHECK THE POSITIONS OF THE ELEVATOR
		elevator.setDefaultCommand(new ElevateJoystick(elevator, operator));
		arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
		// wrist.setDefaultCommand(new MoveWristManual(wrist, operator));

		// new JoystickButton(operator, 1).whenHeld(new ElevatePosition(elevator, 34.45866374));
		// new JoystickButton(operator, 2).whenHeld(new ElevatePosition(elevator, 18.6));
		// new JoystickButton(operator,3).whenHeld(new ElevatePosition(elevator, 2.735));
		new JoystickButton(operator, 4).whenHeld(new ElevatePosition(elevator, -13.13));
		new JoystickButton(operator, 1).whenHeld(new MoveArmPosition(arm, -6));
		new JoystickButton(operator, 2).whenHeld(new MoveArmPosition(arm, -50));
		new JoystickButton(operator, 3).whenHeld(new MoveArmPosition(arm, -80));

		// new JoystickButton(operator, 5).whenPressed(new SetElevatorBase(elevator));

	}

	// AUTON
	public Command getAutonomousCommand() {
		
		return chooser.getSelected();
	}

	// 	// Create Trajectory Speed/Settings
	// 	TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
	// 		Constants.Drivetrain.kMaxSpeedMetersPerSecond,
	// 			Constants.Drivetrain.kTeleDriveMaxAccelerationUnitsPerSecond)
	// 			.setKinematics(Constants.Drivetrain.driveKinematics);

	// 	// Create auton trajectory
	// 	Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
	// 		// Initial point/rotation
	// 		new Pose2d(0, 0, new Rotation2d(0)),
	// 		// Points to traverse to
	// 		List.of(
	// 			new Translation2d(1, 0),
	// 			new Translation2d(1, -1)
	// 		),
	// 		// Final point + rotation
	// 		new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
	// 		trajectoryConfig
	// 	);
	// 	// return trajectory;
	// // }

	// 	// PID Controller for tracking trajectory (profiled = limits max speed/rot)
	// 	// PIDController xController = new PIDController(Constants.Drivetrain.PID_CONTROLLER_X_P, 0, 0);
	// 	// PIDController yController = new PIDController(Constants.Drivetrain.PID_CONTROLLER_Y_P, 0, 0);
	// 	// ProfiledPIDController tController = new ProfiledPIDController(Constants.Drivetrain.PID_CONTROLLER_T_P, 0, 0,
	// 	// 	new TrapezoidProfile.Constraints(
	// 	// 		Constants.Drivetrain.kPhysicalMaxAngularSpeedRadiansPerSecond,
	// 	// 		Constants.Drivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond));
	// 	// tController.enableContinuousInput(-Math.PI, Math.PI);

		// Follow trajectory command
		// SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
		// 	trajectory,
		// 	swerveDrive::getPose,
		// 	Constants.Drivetrain.driveKinematics,
		// 	xController,
		// 	yController,
		// 	tController,
		// 	swerveDrive::setModuleStates,
		// 	swerveDrive
		// );

		// Inits, wrapup, returns
		// return new SequentialCommandGroup(
		// 	new InstantCommand(() -> swerveDrive.resetOdometer(trajectory.getInitialPose())), 
		// 	swerveControllerCommand, 
		// 	new InstantCommand(() -> swerveDrive.stopModules())
		// );

	// }	@Override
}
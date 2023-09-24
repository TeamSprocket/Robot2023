	package frc.robot;

	import com.ctre.phoenix.motorcontrol.*;

	import edu.wpi.first.cameraserver.CameraServer;
	import edu.wpi.first.cscore.CameraServerCvJNI;
	import edu.wpi.first.cscore.CvSink;
	import edu.wpi.first.wpilibj.GenericHID;
	import edu.wpi.first.wpilibj.PowerDistribution;
	import edu.wpi.first.wpilibj.Timer;
	import edu.wpi.first.wpilibj.XboxController;
	import edu.wpi.first.wpilibj.GenericHID.RumbleType;
	import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
	import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
	import edu.wpi.first.wpilibj2.command.Command;
	import edu.wpi.first.wpilibj2.command.button.JoystickButton;
	import frc.robot.commands.persistent.RollClaw;
	import frc.robot.commands.persistent.SwerveDriveCmd;

	import frc.robot.commands.*;
	import frc.robot.commands.auton.*;
	import frc.robot.commands.persistent.*;
	import frc.robot.commands.superstructure.*;


	import frc.robot.subsystems.*;

	import frc.robot.commands.auton.*;
	import frc.robot.subsystems.Elevator;
	import frc.robot.subsystems.Arm;
	import frc.robot.subsystems.Intake;
	import frc.robot.subsystems.SwerveDrive;
	import frc.robot.subsystems.Wrist;
	import frc.robot.subsystems.LEDStrip;
	import frc.robot.subsystems.Superstructure;
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
		XboxController[] controllers = {driver, operator};

		//Smartdashboard
		//Subsystems
		private final SwerveDrive swerveDrive = new SwerveDrive();
		private final PowerDistribution pdh = new PowerDistribution();

		Superstructure superstructure = new Superstructure();
		// private final LEDStrip ledStrip = new LEDStrip();

		
		public RobotContainer() {}	

		// --------------------=Auton Selection=--------------------
public Command getAutonomousCommand() {
	// return new AutonOneCubeOnly(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Universal 
	// return new AutonDoNothing();
	// return new AutonOneCube(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonTwoCube(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Middle (locbvb  ation bot starts from POV of drivers)
	return new AutonBloopBalance(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonBloopBalanceReverse(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonOneHighCubeBalance(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Right (location bot starts from POV of drivers)
	// return new AutonOneHighCubeBalanceLeft(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Left (location bot starts from POV of drivers)
	// return new AutonOneHighCubeBalanceRight(swerveDrive, elevator, arm, wrist, claw);



}
		


		public void configureButtonBindings() {
			// --------------------=Driver=--------------------
			swerveDrive.setDefaultCommand(new SwerveDriveCmd(
				swerveDrive, 
				// X
				() -> -driver.getLeftY(), 
				// Y
				() -> driver.getLeftX(), 
				// T
				() -> -driver.getRightX()));
			claw.setDefaultCommand(new RollClaw(claw, driver));
			new JoystickButton(driver, RobotMap.Controller.RB).whenPressed(() -> swerveDrive.zeroHeading());
			new JoystickButton(driver, RobotMap.Controller.LB).whenPressed(() -> swerveDrive.togglePrecise());
			new JoystickButton(driver, RobotMap.Controller.Y).whenPressed(() -> swerveDrive.updateHeading(Direction.FRONT));
			new JoystickButton(driver, RobotMap.Controller.X).whenPressed(() -> swerveDrive.updateHeading(Direction.LEFT));
			new JoystickButton(driver, RobotMap.Controller.B).whenPressed(() -> swerveDrive.updateHeading(Direction.RIGHT));
			new JoystickButton(driver, RobotMap.Controller.A).whenPressed(() -> swerveDrive.updateHeading(Direction.BACK));
				// new JoystickButton(driver,
			// 	3).whenPressed(() -> swerveDrive.zeroTalonsABS());
			// new JoystickButton(driver, 2).whenHeld(new LimelightAlign(swerveDrive));


			// --------------------=Operator=-------------------- 
			new JoystickButton(operator, 1).onTrue(new SetHome(superstructure));

			new JoystickButton(operator, 2).onTrue(new SetMidCone(superstructure));
			new JoystickButton(operator,3).onTrue(new SetMidCube(superstructure));
			new JoystickButton(operator, 4).onTrue(new SetHighCone(superstructure));
			new JoystickButton(operator, 5).onTrue(new SetHighCube(superstructure));

			new JoystickButton(operator, 6).onTrue(new SetIntakeCone(superstructure));
			new JoystickButton(operator, 7).onTrue(new SetIntakeFloorCone(superstructure));
			new JoystickButton(operator, 8).onTrue(new SetIntakeCube(superstructure));

			new JoystickButton(operator, 9).onTrue(new SetManual(superstructure));
			new JoystickButton(operator, 10).onTrue(new SetOff(superstructure));
		}

		public CvSink getCameraFeed() {
			return CameraServer.getVideo();
		}
		

		public void rumbleControllers(double rumbleValue) {
			for (XboxController controller : controllers) {
				controller.setRumble(RumbleType.kBothRumble, rumbleValue);
			}
		}

		public void initRumbleTimer() {
			// 30 sec warning
			// new VibrateControllerTimed(controllers, 135 - 30, 0.25);
			// new VibrateControllerTimed(controllers, 135 - 30 - 0.5, 0.25);
			// new VibrateControllerTimed(controllers, 135 - 30 - 0.5 - 0.5, 0.25);

			// // 20 sec warning
			// new VibrateControllerTimed(controllers, 135 - 20, 0.25);
			// new VibrateControllerTimed(controllers, 135 - 20 - 0.5, 0.25);
			
			// // 10 sec warning
			// new VibrateControllerTimed(controllers, 135 - 10, 1);
			
		}


		// Methods
		public void autonInit() {
			swerveDrive.zeroTalons();
			// swerveDrive.calibrateGyro();
			swerveDrive.zeroHeading();	
		}

		public SwerveDrive getSwerveDrive() {
			return swerveDrive;
		}

		public void clearStickyFaults() {
			pdh.clearStickyFaults();
			superstructure.clearStickyFaults();
			// swerveDrive.clearStickyFaults();
		}

		public void setSwerveDriveCurrentLimitTurn(double currentLimit) {
			swerveDrive.setCurrentLimitTurn(currentLimit);
		}

		public void setSwerveDriveCurrentLimitDrive(double currentLimit) {
			swerveDrive.setCurrentLimitDrive(currentLimit);
		}


		public void setTurnDefaultMode(NeutralMode mode) {
			swerveDrive.setTurnDefaultMode(mode);
		}

		public void setDriveDefaultMode(NeutralMode mode) {
			swerveDrive.setDriveDefaultMode(mode);
		}

		public double headingOffset() {
			return Math.abs(swerveDrive.getHeadingRad() - Math.PI);
		}

		public double booleanToDouble(boolean value) {
			if (value) {
				return 1.0;
			}
			return 0.0;
		}




			
			
		

		// 	// Create Trajectory Speed/Settings
		// 	TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
		// 		Constants.Drivetrain.kMaxSpeed,
		// 			Constants.Drivetrain.kMaxAccel)
		// 			.setKinematics(Constants.Drivetrain.kDriveKinematics);

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
		// 	// 		Constants.Drivetrain.kMaxTurnSpeed,
		// 	// 		Constants.Drivetrain.kMaxTurnAccel));
		// 	// tController.enableContinuousInput(-Math.PI, Math.PI);

			// Follow trajectory command
			// SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			// 	trajectory,
			// 	swerveDrive::getPose,
			// 	Constants.Drivetrain.kDriveKinematics,
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





		public void outputPitch() {
			// System.out.println(Math.abs(swerveDrive.getPitchDeg()));
			// SmartDashboard.putNumber("Pitch Angle Balance", swerveDrive.getPitchDeg());
		}






		
	}



	
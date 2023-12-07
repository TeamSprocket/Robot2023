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
import frc.robot.commands.macro.LimelightAlign;
	import frc.robot.commands.macro.ResetEncoders;
	import frc.robot.commands.macro.SetDeport;
	import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.SetHighCube;
import frc.robot.commands.macro.SetHeightElevaror;
import frc.robot.commands.macro.SetHome;
	import frc.robot.commands.macro.SetLowConeTilted;
	import frc.robot.commands.macro.SetLowCube;
import frc.robot.commands.macro.SetElevatorInit;
import frc.robot.commands.macro.SetMid;
import frc.robot.commands.macro.SetMidCube;
import frc.robot.commands.macro.timed.PIDTurnTimed;
import frc.robot.commands.macro.SetHumanPlayer;
	import frc.robot.commands.macro.SetLowConeStanding;
	import frc.robot.commands.persistent.Elevate;
	import frc.robot.commands.persistent.MoveArmJoystick;
	import frc.robot.commands.persistent.MoveWristManual;
	import frc.robot.commands.persistent.RollClaw;
	import frc.robot.commands.persistent.SwerveDriveCmd;
// import frc.robot.commands.auton.AutonBalance;
import frc.robot.commands.auton.*;
// import frc.robot.commands.persistent.VibrateControllerTimed;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorReWrite;
import frc.robot.subsystems.Arm;
	import frc.robot.subsystems.Claw;
	import frc.robot.subsystems.SwerveDrive;
	import frc.robot.subsystems.Wrist;
	import frc.robot.subsystems.LEDStrip;
	/**
	 * This class is where the bulk of the robot should be declared. Since
	 * Command-based is a "declarative" paradigm, very little robot logic should
	 * actually be handled in the {@link Robot} periodic methods (other than the
	 * scheduler calls). Instead, the structure of the robot (including subsystems,
	 * commands, and button mappings) should be declared here.
	 */
	public final class RobotContainer {
		Timer timer;
		double last = 0.0;
		double[] desiredStates = new double[13];

		//Controllers
		private final XboxController driver = new XboxController(0);
		private final XboxController operator = new XboxController(1);
		XboxController[] controllers = {driver, operator};

		//Smartdashboard
		//Subsystems
		private final SwerveDrive swerveDrive = new SwerveDrive();
		private final ElevatorReWrite elevator = new ElevatorReWrite();
		// private final Elevator elevator = new Elevator();
		private final Arm arm = new Arm();
		private final Wrist wrist = new Wrist();
		private final Claw claw = new Claw();
		private final PowerDistribution pdh = new PowerDistribution();
		// private final LEDStrip ledStrip = new LEDStrip();

		
		public RobotContainer() {	
			this.timer = new Timer();
			timer.reset(); 
		}	

		// --------------------=Auton Selection=--------------------
public Command getAutonomousCommand() {
	// return new AutonOneCubeOnly(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Universal 
	return new AutonDoNothing();
	// return new AutonOneCube(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonTwoCube(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Middle (locbvb  ation bot starts from POV of drivers)
	// return new AutonBloopBalance(swerveDrive, elevator, arm, wrist, claw);
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
			new JoystickButton(driver,
				RobotMap.Controller.RESET_GYRO_HEADING_BUTTON_ID).whenPressed(() -> swerveDrive.zeroHeading());
			new JoystickButton(driver, 8).whenPressed(() -> swerveDrive.togglePrecise());
				// new JoystickButton(driver,
			// 	3).whenPressed(() -> swerveDrive.zeroTalonsABS());
			new JoystickButton(driver, 2).whenHeld(new LimelightAlign(swerveDrive));


			// --------------------=Operator=-------------------- ,
			// elevator.setDefaultCommand(new Elevate(elevator, operator));
			// arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
			// wrist.setDefaultCommand(new MoveWristManual(wrist, operator));
			// new JoystickButton(operator, 1).whenHeld(new SetMid(elevator, arm, wrist));
			// new JoystickButton(operator, 2).whenHeld(new SetHighCube(elevator, arm, wrist));
			// new JoystickButton(operator, 3).whenHeld(new SetHome(elevator, arm, wrist));
			// new JoystickButton(operator, 4).whenHeld(new SetHigh(elevator, arm, wrist));
			// new JoystickButton(operator, 5).whenHeld(new SetLowCube(elevator, arm, wrist));
			// new JoystickButton(operator, 6).whenHeld(new SetLowConeTilted(elevator, arm, wrist));
			// new JoystickButton(operator, 7).whenHeld(new ResetEncoders(elevator, arm, wrist));
			// new JoystickButton(operator, 8).whenHeld(new SetMidCube(elevator, arm, wrist));
			// new JoystickButton(operator, 9).whenHeld(new SetLowConeStanding(elevator, arm, wrist));
			// new JoystickButton(operator, 10).whenHeld(new SetDeport(elevator, arm, wrist));

			new SetElevatorInit(elevator);
			new JoystickButton(operator, 4).onTrue(new SetHeightElevaror(elevator, "up"));
			new JoystickButton(operator, 3).onTrue(new SetHeightElevaror(elevator, "down"));
			//new JoystickButton(operator, 4).onTrue(new SetHighRewrite(elevator));
			//new JoystickButton(operator, 3).onTrue(new Setlowrewrite(elevator));
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
			timer.start();
			// System.out.println("TELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\nTELEOPINIT\n");
			// new VibrateControllerTimed(controllers, 10, 1);
			double time = timer.get();

			if (time > 105 && time < 106) {
				rumbleControllers(1);
			}
			else if (time > 115 && time < 116) {
				rumbleControllers(1);
			} 
			else if (time > 125 && time < 127) {
				rumbleControllers(1);
			}  
			else {
				rumbleControllers(0);
			}

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
			elevator.clearStickyFaults();
			arm.clearStickyFaults();
			wrist.clearStickyFaults();
			claw.clearStickyFaults();
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

		public void outputAutonLog() {
			timer.start();

			if (swerveDrive.getDesiredStates() != null) {
				desiredStates[0] = swerveDrive.getDesiredStates()[0].speedMetersPerSecond;
				desiredStates[1] = swerveDrive.getDesiredStates()[1].speedMetersPerSecond;
				desiredStates[2] = swerveDrive.getDesiredStates()[2].speedMetersPerSecond;
				desiredStates[3] = swerveDrive.getDesiredStates()[3].speedMetersPerSecond;

				desiredStates[4] = swerveDrive.getDesiredStates()[0].angle.getRadians();
				desiredStates[5] = swerveDrive.getDesiredStates()[1].angle.getRadians();
				desiredStates[6] = swerveDrive.getDesiredStates()[2].angle.getRadians();
				desiredStates[7] = swerveDrive.getDesiredStates()[3].angle.getRadians();
			} else {
				desiredStates[0] = 0;
				desiredStates[1] = 0;
				desiredStates[2] = 0;
				desiredStates[3] = 0;

				desiredStates[4] = 0;
				desiredStates[5] = 0;
				desiredStates[6] = 0;
				desiredStates[7] = 0;
			}

			// desiredStates[8] = booleanToDouble(operator.getRightBumper());
			// desiredStates[9] = booleanToDouble(operator.getAButton());
			// desiredStates[10] = booleanToDouble(operator.getYButton());
			// desiredStates[11] = booleanToDouble(operator.getXButton());

			// desiredStates[12] = (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis());

				double time = Math.round(timer.get() * 10) / 10.0;
				// System.out.println(time);
				if (time - (int) (time) != last) {
					last = time - (int) (time);

					System.out.print("AutonLog: ");
					for (double num : desiredStates) {
						System.out.print(num + " ");
					}
					System.out.println();
				}
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





		public void outputPitch() {
			// System.out.println(Math.abs(swerveDrive.getPitchDeg()));
			SmartDashboard.putNumber("Pitch Angle Balance", swerveDrive.getPitchDeg());
		}






		
	}



	
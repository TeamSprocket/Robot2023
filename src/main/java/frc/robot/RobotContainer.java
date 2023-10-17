package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auton_routines.*;
import frc.robot.commands.macro.*;
import frc.robot.commands.persistent.*;
import frc.robot.commands.superstructure.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrive.Direction;

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
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	private final Wrist wrist = new Wrist();
	private final Claw claw = new Claw();
	private final PowerDistribution pdh = new PowerDistribution();

	SendableChooser<Command> autonChooser = new SendableChooser<>();
	

	// return new AutonOneCubeOnly(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Universal 
	// return new AutonDoNothing();
	// return new AutonBloop(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonHighOneCube(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonTwoCube(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Middle (location bot starts from POV of drivers)
	// return new AutonBloopBalance(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonBloopBalanceReverse(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonOnezzHighCubeBalance(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Right (location bot starts from POV of drivers)
	// return new AutonHighOneCube(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonHighOneCubeNoTaxi(swerveDrive, elevator, arm, wrist, claw);

	/////////////// Left (location bot starts from POV of drivers)
	// return new AutonOneHighCubeBalanceRight(swerveDrive, elevator, arm, wrist, claw);
	// return new AutonHighOneCubeNoTaxiLeft(swerveDrive, elevator, arm, wrist, claw);

	// return new AutonTester(swerveDrive, elevator, arm, wrist, claw);
	// private final LEDStrip ledStrip = new LEDStrip();

	Command autonDoNothing = new AutonDoNothing();
	// Command autonBloopBalance = new AutonBloopBalance(swerveDrive, elevator, arm, wrist, claw);
	Command autonHighCubeBalance = new AutonHighCubeBalance(swerveDrive, elevator, arm, wrist, claw);
	// Command autonHighCubeNoTaxiLeft = new AutonHighOneCubeNoTaxiLeft(swerveDrive, elevator, arm, wrist, claw);
	// Command autonHighCubeNoTaxiRight = new AutonHighOneCubeNoTaxiRight(swerveDrive, elevator, arm, wrist, claw);
	Command autonHighOneCubeLeft = new AutonHighOneCubeLeft(swerveDrive, elevator, arm, wrist, claw);
	Command autonHighOneCubeRight = new AutonHighOneCubeRight(swerveDrive, elevator, arm, wrist, claw);

	// Command autonHighTwoCubeRight = new AutonTwoCubeRight(swerveDrive, elevator, arm, wrist, claw);
	
	// Command autonHighCubeLeft = new ();
	// Command autonHighCubeRight = new AutonHighOneCubeRight();

	
	public RobotContainer() {	
		this.timer = new Timer();
		timer.reset(); 
	}	

	// --------------------=Auton Selection=--------------------
	public void postAutonChoices() {
		autonChooser.addOption("Do Nothing - ALL", autonDoNothing);
		// autonChooser.addOption("Bloop Balance - MID", autonBloopBalance);
		autonChooser.addOption("High Cube Balance - MID", autonHighCubeBalance);
		// autonChooser.addOption("High Cube No Taxi - LEFT", autonHighCubeNoTaxiLeft);
		// autonChooser.addOption("High Cube No Taxi - RIGHT", autonHighCubeNoTaxiRight);
		autonChooser.addOption("High Cube Taxi - LEFT", autonHighOneCubeLeft);
		autonChooser.addOption("High Cube Taxi - RIGHT", autonHighOneCubeRight);

		// autonChooser.addOption("[NOT WORKING] Two High Cube Taxi - RIGHT", autonHighTwoCubeRight);
		
		SmartDashboard.putData(autonChooser);

	}
		
	public Command getAutonomousCommand() {
		return autonChooser.getSelected();
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


		// --------------------=Operator=-------------------- ,
		elevator.setDefaultCommand(new MoveElevatorManual(elevator, operator));
		arm.setDefaultCommand(new MoveArmManual(arm, operator));
		wrist.setDefaultCommand(new MoveWristManual(wrist, operator));
		new JoystickButton(operator, RobotMap.Controller.A).whenHeld(new SetMid(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.B).whenHeld(new SetHighCube(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.X).whenHeld(new SetHome(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.Y).whenHeld(new SetHigh(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LB).whenHeld(new SetLowCube(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.RB).whenHeld(new SetLowConeTilted(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LOGO_LEFT).whenHeld(new ResetEncoders(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LOGO_RIGHT).whenHeld(new SetMidCube(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LEFT_STICK_BUTTON).whenHeld(new SetLowConeStanding(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.RIGHT_STICK_BUTTON).whenHeld(new SetDeport(elevator, arm, wrist));
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

	// public WPI_TalonFX getClawMotorObj() {
		// return claw.getClawMotor();
	// }

	public void resetEncoders() {
		new ResetEncoders(elevator, arm, wrist);
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
		SmartDashboard.putNumber("Pitch Angle Balance", swerveDrive.getPitchDeg());
	}






	
}




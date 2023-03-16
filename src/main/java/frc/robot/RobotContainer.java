package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.commands.auton.BalanceOnChargeStation;
// import frc.robot.commands.auton.DeportArmTimed;
// import frc.robot.commands.auton.BalanceOnChargeStationVertical;
import frc.robot.commands.auton.PIDDriveTimed;
import frc.robot.commands.auton.PIDTurnTimed;
// import frc.robot.commands.auton.ParseAuton;
// import frc.robot.commands.auton.SwerveAutonTest;
import frc.robot.commands.auton.WaitTimed;

import frc.robot.commands.macro.ElevatePosition;
import frc.robot.commands.macro.LimelightAlign;
import frc.robot.commands.macro.MoveArmPosition;
import frc.robot.commands.macro.MoveWristAngle;
import frc.robot.commands.macro.ResetWristEncoder;
import frc.robot.commands.macro.SetDeport;
import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.SetHome;
import frc.robot.commands.macro.SetLowConeTilted;
import frc.robot.commands.macro.SetLowCube;
import frc.robot.commands.macro.SetMid;
import frc.robot.commands.macro.SwerveDriveCmdPrecise;
import frc.robot.commands.macro.ToggleSwervePrecise;
import frc.robot.commands.macro.timed.DeportArm;
import frc.robot.commands.macro.timed.RollClawTimed;
import frc.robot.commands.macro.timed.SetHighTimed;
import frc.robot.commands.macro.timed.SetHomeTimed;
import frc.robot.commands.macro.timed.SetHumanPlayerTimed;
import frc.robot.commands.macro.timed.SetLowCubeTimed;
import frc.robot.commands.macro.timed.SwerveDriveCmdTimed;
import frc.robot.commands.macro.SetHumanPlayer;
import frc.robot.commands.macro.SetLowConeStanding;
import frc.robot.commands.persistent.Elevate;
import frc.robot.commands.persistent.MoveArmJoystick;
import frc.robot.commands.persistent.MoveWristManual;
import frc.robot.commands.persistent.RollClaw;
// import frc.robot.commands.auton.SwerveAutonTest;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
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
	Timer timer;
	double last = 0.0;
	double[] desiredStates = new double[13];

	//Controllers
	private final XboxController driver = new XboxController(0);
	private final XboxController operator = new XboxController(1);

	//Smartdashboard
	//Subsystems

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
		this.timer = new Timer();
		
		SmartDashboard.putData(chooser);
		timer.reset();
	}	

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	// public void calibrateGyro() {
	// 	swerveDrive.calibrateGyro();
	// }

	/**
	 * Use this method to define your button->command mappings.  Buttons can be
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
		// claw.setDefaultCommand(new RollClaw(claw, () -> (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis())));
		claw.setDefaultCommand(new RollClaw(claw, driver));
		
	

		// Elevator
		//TODO CHECK THE POSITIONS OF THE ELEVATOR
		// elevator.setDefaultCommand(new Elevate(elevator, (operator.getLeftTriggerAxis() - operator.getRightTriggerAxis())));
		elevator.setDefaultCommand(new Elevate(elevator, operator));
		arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
		wrist.setDefaultCommand(new MoveWristManual(wrist, operator));
		

		// Swerve Drive (instant command reset heading)
		new JoystickButton(driver,
		 	RobotMap.Controller.RESET_GYRO_HEADING_BUTTON_ID).whenPressed(() -> swerveDrive.zeroHeading());
		new JoystickButton(driver, 2).whenHeld(new LimelightAlign(swerveDrive));
		// new JoystickButton(driver, 2).whenPressed(() -> swerveDrive.zeroTalons());
		// new JoystickButton(driver, 3).whenHeld(new ShootClaw(10));
		
		// new JoystickButton(driver, 7).whenPressed(() -> swerveDrive.zeroTalons());

		new JoystickButton(operator, 1).whenHeld(new SetMid(elevator, arm, wrist));
		new JoystickButton(operator, 2).whenHeld(new SetHumanPlayer(elevator, arm, wrist));
		new JoystickButton(operator, 3).whenHeld(new SetHome(elevator, arm, wrist));
		new JoystickButton(operator, 4).whenHeld(new SetHigh(elevator, arm, wrist));
		new JoystickButton(operator, 5).whenHeld(new SetLowCube(elevator, arm, wrist));
		new JoystickButton(operator, 6).whenHeld(new SetLowConeTilted(elevator, arm, wrist));
		new JoystickButton(operator, 7).whenHeld(new ResetWristEncoder(wrist));
		new JoystickButton(operator, 9).whenHeld(new SetLowConeStanding(elevator, arm, wrist));
		new JoystickButton(operator, 10).whenHeld(new SetDeport(elevator, arm, wrist));

		/* 
		new JoystickButton(operator, 1).whenHeld(new SetLowCone(elevator, arm, wrist));
		new JoystickButton(operator, 2).whenHeld(new SetHumanPlayer(elevator, arm, wrist));
		new JoystickButton(operator, 3).whenHeld(new SetHome(elevator, arm, wrist));
		new JoystickButton(operator, 4).whenHeld(new SetHigh(elevator, arm, wrist));
		new JoystickButton(operator, 5).whenHeld(new SetMid(elevator, arm, wrist));
		new JoystickButton(operator, 6).whenHeld(new DeportArm(elevator, arm, wrist));

		new JoystickButton(operator, 7).whenHeld(new MoveWristAngle(wrist, 10));
		*/
		

		// new POVButton(driver, 90).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 1, 0));
		// new POVButton(driver, 270).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, -1, 0));
		// new POVButton(driver, 0).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 0, 1));
		// new POVButton(driver, 180).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 0, -1));

		// new POVButton(driver, 45).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 1, 1));
		// new POVButton(driver, 135).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, -1, 1));
		// new POVButton(driver, 225).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, -1, -1));
		// new POVButton(driver, 315).whenHeld(new SwerveDriveCmdPrecise(swerveDrive, 1, -1));


	
		// new JoystickButton(operator, 5).whenPressed(new SetElevatorBase(elevator));

	}

	public void autonInit() {
		swerveDrive.zeroTalons();

		swerveDrive.calibrateGyro();
		swerveDrive.zeroHeading();	
	}

	public void clearStickyFaults() {

	}

	public void setSwerveDriveCurrentLimitTurn(double currentLimit) {
		swerveDrive.setCurrentLimitTurn(currentLimit);
	}

	public void setSwerveDriveCurrentLimitDrive(double currentLimit) {
		swerveDrive.setCurrentLimitDrive(currentLimit);
	}


	// AUTON
	// public Command getAutonomousCommand() {
	public Command getAutonomousCommand() {
		// Do nothing (0)
		// return (Command) (new SequentialCommandGroup(
		// ));

		// Move back (4)
		// return (Command) (new SequentialCommandGroup(
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.2, new Rotation2d(0.1)), 4) //4
		// ));

		//Place cone & move back (9.5)
		// return (Command) (new SequentialCommandGroup(
		// 	new DeportArm(elevator, arm, wrist), //2
		// 	new SetHighTimed(elevator, arm, wrist, 2), //2
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.2, new Rotation2d(0.0)), 1), //1
		// 	new ToggleClaw(claw), //0
		// 	new WaitTimed(0.5), //0.5
		// 	new ParallelCommandGroup( //4
		// 		new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.2, new Rotation2d(0.0)), 4), //4*
		// 		new SetHomeTimed(elevator, arm, wrist, 4) //4*
		// 	)
		// ));

		// Place cube & move back (10)
		// return (Command) (new SequentialCommandGroup(
		// 	new DeportArm(elevator, arm, wrist), //2
		// 	new SetHighTimed(elevator, arm, wrist, 2), //2
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.25, new Rotation2d(0.0)), 2), //1
		// 	// new WaitTimed(0.5),
		// 	new RollClawTimed(claw, 0.5, 1), //1
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 1),
		// 	new ParallelCommandGroup(
		// 		new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 3.25),
		// 		new SetHomeTimed(elevator, arm, wrist, 4) //4*
		// 	)
		// ));	
		

		// Place Cube Only
		// return (Command) (new SequentialCommandGroup(
		// 	new DeportArm(elevator, arm, wrist), //2
		// 	new SetHighTimed(elevator, arm, wrist, 2), //2
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.25, new Rotation2d(0.0)), 2), //1
			// // new WaitTimed(0.5),
		// 	new RollClawTimed(claw, 0.5, 1) //1
		// 	// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.25, new Rotation2d(0.0)), 2),
		// 	// new ParallelCommandGroup(
		// 	// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 3.25),
		// 	// new SetHomeTimed(elevator, arm, wrist, 4) //4*
		// 	// )
		// ));	
		

		//MOVE FORWARD ONLY
		// return (Command) (new SequentialCommandGroup(
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new Rotation2d(0.0)), 2)
		// ));

		// Balance Charging Station BLUE (idk)
		// return (Command) (new SequentialCommandGroup(
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new Rotation2d(0.0)), 2),
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(-0.24, 0.0, new Rotation2d(0.0)), 2),
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 1.75), //4
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.01), 0.5))
		// ));pz

		// Balance Charging Station RED (idk)
		// return (Command) (new SequentialCommandGroup(
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new Rotation2d(0.0)), 2),
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.24, 0.0, new Rotation2d(0.0)), 2),
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 1.75), //4
		// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.0, new Rotation2d(0.01), 0.5))
		// ));

		// PID Charging Station (idkkkkkkkk)
		// return (Command) (new SequentialCommandGroup(
			// new OneMeterForward(swerveDrive)
			// new BalanceOnChargeStation(swerveDrive, 0.035)
		// ));


		// Macro Recorder Test
		// return (Command) (new SequentialCommandGroup(
			// new ParseAuton(swerveDrive, elevator, arm, wrist, claw)
			// new PIDTurnTimed(swerveDrive, Math.PI, 3),
			// new BalanceOnChargeStation(swerveDrive, 0.035)
		// ));

		// return (Command) (new SequentialCommandGroup(
		// 	new PIDDriveTimed(swerveDrive, 1)
		// ));

		return (Command) (new SequentialCommandGroup(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new DeportArm(elevator, arm, wrist, 1),
					new ParallelCommandGroup(
						new SetHighTimed(elevator, arm, wrist, 2),
						new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.35, new Rotation2d(0.0)), 1.15)
					)
				),
			new RollClawTimed(claw, 1, 2.5)
			),
			new RollClawTimed(claw, -1, 0.5),
			new ParallelCommandGroup(
						new SetHighTimed(elevator, arm, wrist, 2),
						new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.5, new Rotation2d(0.0)), 3.75)
			),
			new SetHomeTimed(elevator, arm, wrist, 2)
		));
		// 	new OneMeterForward(swerveDrive)
		// ));


		// return (Command) (new SequentialCommandGroup(
		// 	new DeportArm(elevator, arm, wrist, 1),
		// 	new ParallelCommandGroup(
		// 		new SetHighTimed(elevator, arm, wrist, 1), //2
		// 		new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.25, new Rotation2d(0.0)), 1)
		// 	),
		// 	new RollClawTimed(claw, 0.5, 1), //1
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.25, new Rotation2d(0.0)), 0.5),
		// 	new ParallelCommandGroup(
		// 		new SetHomeTimed(elevator, arm, wrist, 1.5),
		// 		new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.25, new Rotation2d(0.0)), 1.5)
		// 	),
		// 	new PIDTurnTimed(swerveDrive, Math.PI, 1),
		// 	new ParallelCommandGroup(
		// 		new SetLowCube(elevator, arm, wrist),
		// 		new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 1.5),
		// 		new RollClawTimed(claw, 1)
		// 	),
		// 	new PIDTurnTimed(swerveDrive, Math.PI, 1),
		// 	new ParallelCommandGroup(
		// 		new SetMid(elevator, arm, wrist),
		// 		new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, 0.3, new Rotation2d(0.0)), 3.5)
		// 	),
		// 	new RollClawTimed(claw, 1)

			
		// 	// new WaitTimed(0.5),
		// 	// new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.25, new Rotation2d(0.0)), 2),
		// 	// new ParallelCommandGroup(
		// 	// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0.0, -0.3, new Rotation2d(0.0)), 3.25),
		// 	// new SetHomeTimed(elevator, arm, wrist, 4) //4*
		// 	// )
		// ));	

		// return (Command) (new SequentialCommandGroup(
			// new PIDTurnTimed(swerveDrive, Math.PI, 3),
			// new PIDDriveTimed(swerveDrive, 2, 0, 5),
			// new PIDTurnTimed(swerveDrive, Math.PI, 3),
			// new PIDDriveTimed(swerveDrive, -2, 0,5)
		
		// ));
		// return (Command) (new SequentialCommandGroup(
			// new ParseAuton(swerveDrive, elevator, arm, wrist, claw),
			// // new PIDTurnTimed(swerveDrive, Math.PI, 3),
			// new BalanceOnChargeStation(swerveDrive, 0.035)
		// ));


		












		//////////////////////////////////////////


		// Command autonRoutine = new SequentialCommandGroup(
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, 0.2, new Rotation2d(0.0)), 1),
		// 	new WaitTimed(3),
		// 	new SwerveDriveCmdTimed(swerveDrive, new Pose2d(0, -0.2, new Rotation2d(0.0)), 1)
		// );
		// return autonRoutine;

				// new DeportArm(elevator, arm, wrist),
				// new SetHighTimed(elevator, arm, wrist, 3),
				// new SetHumanPlayerTimed(elevator, arm, wrist, 2),
				// new RollClawTimed(claw, 1, 1),
				// new ParallelCommandGroup(
				// 	new SetLowTimed(elevator, arm, wrist, 5),
				// 	new RollClawTimed(claw, -0.5, 5)
				// ),
				// new SetHighTimed(elevator, arm, wrist, 3),
				// new RollClawTimed(claw, 1, 1),
				// new SetHomeTimed(elevator, arm, wrist, 3)
		
		
		// return chooser.getSelected();

	}

	public void setTurnDefaultMode(NeutralMode mode) {
		swerveDrive.setTurnDefaultMode(mode);
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

		desiredStates[8] = booleanToDouble(operator.getRightBumper());
		desiredStates[9] = booleanToDouble(operator.getAButton());
		desiredStates[10] = booleanToDouble(operator.getYButton());
		desiredStates[11] = booleanToDouble(operator.getXButton());

		desiredStates[12] = (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis());

		


		
		
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
}
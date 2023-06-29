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

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LEDStrip;

import frc.robot.commands.auton.*;
import frc.robot.commands.auton_sequences.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.limelight.*;
import frc.robot.commands.macro.TogglePrecise;
import frc.robot.commands.macro.ZeroHeading;
import frc.robot.commands.persistent.*;
import frc.robot.commands.persistent.Elevate;
import frc.robot.commands.persistent.MoveArmJoystick;
import frc.robot.commands.persistent.MoveWristManual;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.commands.persistent.DriveTeleop;


public final class RobotContainer {
	// Controllers
	private final XboxController driver = new XboxController(InputMap.Primary.PORT_NUMBER);
	private final XboxController operator = new XboxController(InputMap.Secondary.PORT_NUMBER);

	// Subsystems
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	private final Wrist wrist = new Wrist();
	private final Claw claw = new Claw();
	private final PowerDistribution pdh = new PowerDistribution();

	// Auton Chooser
	private final SendableChooser<Command> autonChooser = new SendableChooser<>();

	public RobotContainer() {
		autonChooser.setDefaultOption("Do Nothing", new AutonDoNothing());
		// autonChooser.addOption(name, command);

		SmartDashboard.putData(autonChooser);
	}

	public Command getAutonomousCommand() {
		return autonChooser.getSelected();
	}

	public void configureButtonBindings() {
		// Primary
		swerveDrive.setDefaultCommand(new DriveTeleop(
				swerveDrive,
				// X-axis
				() -> -driver.getLeftY(),
				// Y-axis
				() -> driver.getLeftX(),
				// T (turn)
				() -> -driver.getRightX()));
		claw.setDefaultCommand(new RollClaw(claw, driver));

		new JoystickButton(driver, InputMap.Primary.RESET_GYRO_HEADIN_BUTTON_ID)
			.onTrue(new ZeroHeading(swerveDrive));
		new JoystickButton(driver, InputMap.Primary.TOGGLE_SWERVE_PRECISE_BUTTON_ID)
			.onTrue(new TogglePrecise(swerveDrive));
		new JoystickButton(driver, InputMap.Primary.LIMELIGHT_ALIGN_BUTTON_ID)
			.whileTrue(new LimelightAlign(swerveDrive));
		// new JoystickButton(driver, InputMap.Primary.RESET_TURN_ENCODER_BUTTON_ID)
			// .onTrue(new ResetTurnEncoders(swerveDrive));


		// Secondary
		elevator.setDefaultCommand(new Elevate(elevator, operator));
		arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
		wrist.setDefaultCommand(new MoveWristManual(wrist, operator));

		new JoystickButton(operator, InputMap.Secondary.SET_MID_CONE_BUTTON_ID)
			.onTrue(new SetMid(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_HIGH_CUBE_BUTTON_ID)
			.onTrue(new SetHighCube(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_HOME_BUTTON_ID)
			.onTrue(new SetHome(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_HIGH_CONE_BUTTON_ID)
			.onTrue(new SetHigh(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_LOW_CONE_BUTTON_ID)
			.onTrue(new SetLowCube(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_FLOOR_CONE_BUTTON_ID)
			.onTrue(new SetLowConeTilted(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.RESET_INTAKE_ENCODERS_BUTTON_ID)
			.onTrue(new ResetIntakeEncoders(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_MID_CUBE_BUTTON_ID)
			.onTrue(new SetMidCube(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_LOW_CONE_BUTTON_ID)
			.onTrue(new SetLowConeStanding(elevator, arm, wrist));
		new JoystickButton(operator, InputMap.Secondary.SET_DEPORT_INTAKE_BUTTON_ID)
			.onTrue(new SetDeport(elevator, arm, wrist));
	}

	// Methods
	
	public void autonInit() {
		swerveDrive.zeroTalons();
		swerveDrive.zeroHeading();
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public void setIsTeleop(boolean isTeleop) {
		Constants.kIsTeleop = isTeleop;
	}

	public void clearStickyFaults() {
		pdh.clearStickyFaults();
	}

	public void setTurnDefaultMode(NeutralMode mode) {
		swerveDrive.setTurnDefaultMode(mode);
	}

	public void setDriveDefaultMode(NeutralMode mode) {
		swerveDrive.setDriveDefaultMode(mode);
	}

	// public void rumbleControllers(double rumbleValue) {
	// 	for (XboxController controller : controllers) {
	// 		controller.setRumble(RumbleType.kBothRumble, rumbleValue);
	// 	}
	// }

	// public void updateRumbleTimer() {}


}

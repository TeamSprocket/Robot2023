package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.macro.ResetEncoders;
import frc.robot.commands.macro.SetDeport;
import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.SetHighCube;
import frc.robot.commands.macro.SetHome;
import frc.robot.commands.macro.SetLowConeTilted;
import frc.robot.commands.macro.SetLowCube;
import frc.robot.commands.macro.SetMid;
import frc.robot.commands.macro.SetMidCube;
import frc.robot.commands.macro.SetLowConeStanding;
import frc.robot.commands.persistent.Elevate;
import frc.robot.commands.persistent.MoveArmJoystick;
import frc.robot.commands.persistent.MoveWristManual;
import frc.robot.commands.persistent.RollClaw;
import frc.robot.commands.persistent.SwerveDriveCmd;
import frc.robot.commands.auton.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SwerveDrive.Direction;

public final class RobotContainer {
	Timer timer;
	double last = 0.0;
	double[] desiredStates = new double[13];

	// Controllers
	private final XboxController driver = new XboxController(0);
	private final XboxController operator = new XboxController(1);
	XboxController[] controllers = { driver, operator };

	// Smartdashboard
	// Subsystems
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	private final Wrist wrist = new Wrist();
	private final Claw claw = new Claw();
	private final PowerDistribution pdh = new PowerDistribution();

	public RobotContainer() {
		this.timer = new Timer();
		timer.reset();
	}

	// --------------------=Auton Selection=--------------------
	public Command getAutonomousCommand() {
		return new AutonDoNothing();
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

		// --------------------=Operator=-------------------- ,
		elevator.setDefaultCommand(new Elevate(elevator, operator));
		arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
		wrist.setDefaultCommand(new MoveWristManual(wrist, operator));
		new JoystickButton(operator, RobotMap.Controller.A).whenHeld(new SetMid(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.B).whenHeld(new SetHighCube(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.X).whenHeld(new SetHome(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.Y).whenHeld(new SetHigh(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LB).whenHeld(new SetLowCube(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.RB).whenHeld(new SetLowConeTilted(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LOGO_LEFT).whenHeld(new ResetEncoders(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LOGO_RIGHT).whenHeld(new SetMidCube(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.LEFT_STICK_BUTTON)
				.whenHeld(new SetLowConeStanding(elevator, arm, wrist));
		new JoystickButton(operator, RobotMap.Controller.RIGHT_STICK_BUTTON)
				.whenHeld(new SetDeport(elevator, arm, wrist));
	}

	public void rumbleControllers(double rumbleValue) {
		for (XboxController controller : controllers) {
			controller.setRumble(RumbleType.kBothRumble, rumbleValue);
		}
	}

	// Methods
	public void autonInit() {
		swerveDrive.zeroTalons();
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

		double time = Math.round(timer.get() * 10) / 10.0;

		if (time - (int) (time) != last) {
			last = time - (int) (time);

			System.out.print("AutonLog: ");
			for (double num : desiredStates) {
				System.out.print(num + " ");
			}
			System.out.println();
		}
	}
}

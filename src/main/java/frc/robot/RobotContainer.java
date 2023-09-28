package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.macro.ResetEncoders;
import frc.robot.commands.macro.SetDeport;
import frc.robot.commands.macro.SetHigh;
import frc.robot.commands.macro.SetHighCube;
import frc.robot.commands.macro.SetHome;
import frc.robot.commands.macro.SetLowConeTilted;
import frc.robot.commands.macro.SetLowCube;
import frc.robot.commands.macro.SetMid;
import frc.robot.commands.macro.SetMidCube;
import frc.robot.commands.macro.SwerveUtils;
import frc.robot.commands.macro.SwerveUtils.SwerveUtilsCommands;
import frc.robot.commands.macro.SetLowConeStanding;
import frc.robot.commands.persistent.Elevate;
import frc.robot.commands.persistent.MoveArmJoystick;
import frc.robot.commands.persistent.MoveWristManual;
import frc.robot.commands.persistent.RollIntake;
import frc.robot.commands.persistent.SwerveDriveCmd;
import frc.robot.commands.auton.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

public final class RobotContainer {

	private final CommandXboxController driver = new CommandXboxController(0);
	private final CommandXboxController operator = new CommandXboxController(1);
	// CommandXboxController[] controllers = { driver, operator };

	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	private final Wrist wrist = new Wrist();
	private final Intake intake = new Intake();
	private final PowerDistribution pdh = new PowerDistribution();

	SendableChooser<Command> autonChooser = new SendableChooser<>();

	public RobotContainer() {
		autonChooser.addOption("DoNothing", new AutonDoNothing());
		autonChooser.addOption("BloopBalance", new AutonBloopBalance(swerveDrive, elevator, arm, wrist, intake));
		autonChooser.addOption("OneHighCube", new AutonOneHighCube(swerveDrive, elevator, arm, wrist, intake));
		autonChooser.addOption("OneHighCubeBalance", new AutonOneHighCubeBalance(swerveDrive, elevator, arm, wrist, intake));
		autonChooser.addOption("OneHighCubeOneStowCube", new AutonOneHighCubeOneStowCube(swerveDrive, elevator, arm, wrist, intake));
		SmartDashboard.putData(autonChooser);
	}

	// --------------------=Auton Selection=--------------------
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
		intake.setDefaultCommand(new RollIntake(intake, driver));
		driver.button(RobotMap.Controller.RB).onTrue(new SwerveUtils(swerveDrive, 
			SwerveUtilsCommands.ZERO_HEADING));
		driver.button(RobotMap.Controller.LB).onTrue(new SwerveUtils(swerveDrive, 
			SwerveUtilsCommands.TOGGLE_PRECISE));
		driver.button(RobotMap.Controller.Y).onTrue(new SwerveUtils(swerveDrive, 
			SwerveUtilsCommands.UPDATE_HEADING_FRONT));
		driver.button(RobotMap.Controller.X).onTrue(new SwerveUtils(swerveDrive, 
			SwerveUtilsCommands.UPDATE_HEADING_LEFT));
		driver.button(RobotMap.Controller.B).onTrue(new SwerveUtils(swerveDrive, 
			SwerveUtilsCommands.UPDATE_HEADING_RIGHT));
		driver.button(RobotMap.Controller.A).onTrue(new SwerveUtils(swerveDrive, 
			SwerveUtilsCommands.UPDATE_HEADING_BACK));

		// --------------------=Operator=-------------------- ,
		elevator.setDefaultCommand(new Elevate(elevator, operator));
		arm.setDefaultCommand(new MoveArmJoystick(arm, operator));
		wrist.setDefaultCommand(new MoveWristManual(wrist, operator));
		
		operator.button(RobotMap.Controller.A).whileTrue(new SetMid(elevator, arm, wrist));
		operator.button(RobotMap.Controller.B).whileTrue(new SetHighCube(elevator, arm, wrist));
		operator.button(RobotMap.Controller.X).whileTrue(new SetHome(elevator, arm, wrist));
		operator.button(RobotMap.Controller.Y).whileTrue(new SetHigh(elevator, arm, wrist));
		operator.button(RobotMap.Controller.LB).whileTrue(new SetLowCube(elevator, arm, wrist));
		operator.button(RobotMap.Controller.RB).whileTrue(new SetLowConeTilted(elevator, arm, wrist));
		operator.button(RobotMap.Controller.LOGO_LEFT).whileTrue(new ResetEncoders(elevator, arm, wrist));
		operator.button(RobotMap.Controller.LOGO_RIGHT).whileTrue(new SetMidCube(elevator, arm, wrist));
		operator.button(RobotMap.Controller.LEFT_STICK_BUTTON).whileTrue(new SetLowConeStanding(elevator, arm, wrist));
		operator.button(RobotMap.Controller.RIGHT_STICK_BUTTON).whileTrue(new SetDeport(elevator, arm, wrist));
	}

	// Methods
	public void autonInit() {
		swerveDrive.zeroTalons();
		swerveDrive.zeroHeading();
	}

	public void clearStickyFaults() {
		pdh.clearStickyFaults();
		elevator.clearStickyFaults();
		arm.clearStickyFaults();
		wrist.clearStickyFaults();
		intake.clearStickyFaults();
	}

	public void setSwerveTurnMotorDefaultMode(NeutralMode mode) {
		swerveDrive.setSwerveTurnMotorDefaultMode(mode);
	}

	public void setSwerveDriveMotorDefaultMode(NeutralMode mode) {
		swerveDrive.setSwerveDriveMotorDefaultMode(mode);
	}

	// public void rumbleControllers(double rumbleValue) {
	// 	for (CommandXboxController controller : controllers) {
	// 		controller.setRumble(RumbleType.kBothRumble, rumbleValue);
	// 	}
	// }

	
}

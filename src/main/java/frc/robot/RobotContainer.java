package frc.robot;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auton.*;
import frc.robot.commands.macros.ResetGyro;
import frc.robot.commands.macros.ZeroSwerveABS;
import frc.robot.commands.persistent.Drive;
import frc.robot.subsystems.SwerveDrive;


public final class RobotContainer {
	private final XboxController driver = new XboxController(0);
	// private final XboxController operator = new XboxController(1);
	private final SwerveDrive swerveDrive = new SwerveDrive();

	public Command getAutonomousCommand() {
		return new AutonDoNothing();
	}
	
	public RobotContainer() {}	

	public void configureButtonBindings() {
		swerveDrive.setDefaultCommand(new Drive(
			swerveDrive, 
			// X
			() -> -driver.getLeftY(), 
			// Y
			() -> driver.getLeftX(), 
			// T
			() -> -driver.getRightX()));
		new JoystickButton(driver, RobotMap.Controller.RESET_GYRO_HEADING_BUTTON_ID).onTrue(new ResetGyro(swerveDrive));
		new JoystickButton(driver, 3).onTrue(new ZeroSwerveABS(swerveDrive));
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
		swerveDrive.clearStickyFaults();
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




}



	
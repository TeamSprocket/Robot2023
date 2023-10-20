package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ZeroTurn;
import frc.robot.commands.persistent.SwerveDriveCmd;
import frc.robot.commands.ZeroTurnABS;
import frc.robot.subsystems.SwerveDrive;

public final class RobotContainer {
	//Controllers
	private final CommandXboxController driver = new CommandXboxController(0);
	
	//Smartdashboard
	//Subsystems
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final PowerDistribution pdh = new PowerDistribution();
	
	public RobotContainer() {}	

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
			driver.x().onTrue(new ZeroTurn(swerveDrive));
			driver.a().onTrue(new ZeroTurnABS(swerveDrive));
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public void clearStickyFaults() {
		pdh.clearStickyFaults();
		swerveDrive.clearStickyFaults();
	}
	
}




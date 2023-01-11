package frc.robot;

//general imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.util.commands.InstantCommand;
import frc.util.commands.MacroCommand;

//auton imports


//import subsystems


//import commands


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public final class RobotContainer {
    //instantiate subsystems
    //private final Drivetrain drivetrain = new Drivetrain();
	

    //instantiate auton commands
	
	
    //shuffleboard auton commands
	SendableChooser<Command> chooser = new SendableChooser<>();  


	public RobotContainer() {
		
        configureButtonBindings();

        /*assign auton to shuffleboard
        chooser.setDefaultOption(String name, auton)
        chooser.addOption(String name, auton)
        */
		

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
        /*drivetrain
		drivetrain.setDefaultCommand(new Drive(drivetrain, leftJoystick, rightJoystick));
		*/

		//gamepad commands
		//new JoystickButton(gamepad, BUTTONNUMBER).whenPressed(NEW COMMAND(SUBSYSTEMS));
		//subsystem.setDefaultCommand(NEW COMMAND(SUBSYSTEMS, GAMEPAD));
	}

	public Command getAutonomousCommand() {
			return chooser.getSelected();
	}
}
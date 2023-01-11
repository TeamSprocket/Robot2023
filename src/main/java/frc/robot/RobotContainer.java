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
import frc.robot.commands.auton.OneBall;


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
    private final Drivetrain drivetrain = new Drivetrain();
	

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
        
		drivetrain.setDefaultCommand(new Drive(drivetrain, leftJoystick, rightJoystick));
		new JoystickButton(gamepad, 4).whenPressed(new ToggleCompressor(pch, gamepad));
		new JoystickButton(gamepad, 6).whenPressed(new ActuateClimb(climber, true));
		new JoystickButton(gamepad, 5).whenPressed(new ActuateClimb(climber, false));
		new JoystickButton(gamepad, 3).whenPressed(new ToggleIntake(intake));
		new JoystickButton(gamepad, 2).whenHeld(new ShootAuto(shooter, feeder, 0));
		new JoystickButton(gamepad, 7).whenHeld(new FeedManualTop(feeder, gamepad));
		new JoystickButton(gamepad, 8).whenHeld(new FeedManualBot(feeder, gamepad));
    	new JoystickButton(gamepad, 1).whenHeld(new ShootAutoCompressed(shooter, feeder, 0));
		// new JoystickButton(gamepad, 10).whenHeld(new SENIT(shooter, feeder, intake, 0));
		// shooter.setDefaultCommand(new RollShooterManual(shooter, gamepad));
		feeder.setDefaultCommand(new FeedManual(feeder, gamepad));
		intake.setDefaultCommand(new RollIntakeManual(intake, gamepad));
		ledStrip.setDefaultCommand(new BlingBling(ledStrip, shooter));
		climber.setDefaultCommand(new ClimbArmManual(climber, gamepad));
	}
	public Command getAutonomousCommand() {
			return chooser.getSelected();
	}
}
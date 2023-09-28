
package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot {
    private final RobotContainer robotContainer = new RobotContainer();

    public Robot() {}


    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.clearStickyFaults();
    }


    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        Constants.isEnabled = false;
        robotContainer.setSwerveTurnMotorDefaultMode(NeutralMode.Coast);
        robotContainer.setSwerveDriveMotorDefaultMode(NeutralMode.Coast);
    }

    @Override
    public void disabledPeriodic() {}



    @Override
    public void autonomousInit() {
        Constants.isEnabled = false;
        robotContainer.setSwerveTurnMotorDefaultMode(NeutralMode.Brake);
        robotContainer.setSwerveDriveMotorDefaultMode(NeutralMode.Brake);
        robotContainer.autonInit();

        if (robotContainer.getAutonomousCommand() != null) {
            robotContainer.getAutonomousCommand().schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}



    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        
        Constants.isEnabled = true;
        robotContainer.setSwerveTurnMotorDefaultMode(NeutralMode.Coast); //TODO set brake?
        robotContainer.setSwerveDriveMotorDefaultMode(NeutralMode.Coast);
        robotContainer.configureButtonBindings();
    }

    @Override
    public void teleopPeriodic() { }

}
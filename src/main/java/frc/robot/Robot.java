
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    Command auton;

    private final RobotContainer bot;

    public Robot() {
        bot = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // bot.setSwerveDriveCurrentLimitTurn(Constants.Drivetrain.CURRENT_LIMIT_TURN_MOTOR);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        bot.clearStickyFaults();
    }

    @Override
    public void disabledInit() {
        bot.setIsTeleop(false);

        CommandScheduler.getInstance().cancelAll();
        
        bot.setDriveDefaultMode(NeutralMode.Coast);
        bot.setTurnDefaultMode(NeutralMode.Coast);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        bot.autonInit();
        
        bot.setIsTeleop(false);

        bot.setDriveDefaultMode(NeutralMode.Brake);
        bot.setTurnDefaultMode(NeutralMode.Brake);
        
        this.auton = bot.getAutonomousCommand();

        if (auton!=null) {
            auton.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        bot.setIsTeleop(true);

        bot.configureButtonBindings();

        bot.setTurnDefaultMode(NeutralMode.Brake);
        bot.setDriveDefaultMode(NeutralMode.Brake);
    }
    
    @Override
    public void teleopPeriodic() {
        // bot.updateRumbleTimer();
    }



}
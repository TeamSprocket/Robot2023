package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.util.commands.MacroCommand;

// Used for auton

public class WaitTimed extends MacroCommand {
    private final double seconds;
    private final Timer timer;

    public WaitTimed(double seconds){
        this.seconds = seconds;
        timer = new Timer();
    }

    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
    }

    public boolean isFinished(){
        if (timer.get() >= seconds) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted){
    }

}
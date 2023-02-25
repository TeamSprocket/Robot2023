package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        System.out.println("WAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\nWAITING.....\n");
        SmartDashboard.putString("Auton STATUS", "WAITING...");
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
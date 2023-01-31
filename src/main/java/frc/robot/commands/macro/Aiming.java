package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Drivetrain;
import frc.util.commands.MacroCommand;


public class Aiming extends MacroCommand{

    private final Limelight limelight;
    //private final Drivetrain drivetrain;

    public Aiming(Limelight limelight) {
        this.limelight = limelight;
        // this.drivetrain = drivetrain;
    }

    @Override
    public void initialize(){
        limelight.findDistance();
    }

    @Override
    public void execute(){
        limelight.autoAim();
    }

    @Override
    public boolean isFinished(){
        if (limelight.getTx() == 0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
       //drivetrain.setSpeed(0,0);
    }
    
}

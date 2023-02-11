package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class AutoAim extends MacroCommand{
    

    private final Limelight limelight;
    private final SwerveDrive swerveDrive;

    public AutoAim(Limelight limelight, SwerveDrive swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
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
    }

  
}

package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
//import frc.robot.subsystems.Drivetrain;
import frc.util.commands.MacroCommand;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class Aiming extends MacroCommand{

    private final Limelight limelight;
    private final SwerveDrive swerveDrive;

    public Aiming(Limelight limelight, SwerveDrive swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("distance",limelight.findDistance());
    }

    @Override
    public void execute(){
        new ParallelCommandGroup(
            new AutoAim(limelight, swerveDrive),
            new SwerveDriveCmd(swerveDrive, null, null, null)
        )
;
       
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
    //TODO commands that offset robot after it finds the tape

    
}

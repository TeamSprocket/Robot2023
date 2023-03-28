package frc.robot.commands.macro.timed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.util.commands.MacroCommand;

// Used for auton

public class ZeroHeadingTimed extends MacroCommand {
    private final double seconds;
    private final Timer timer;
    private double iter = 0;
    SwerveDrive swerveDrive;

    public ZeroHeadingTimed(SwerveDrive swerveDrive,double seconds){
        this.seconds = seconds;
        timer = new Timer();
        this.swerveDrive = swerveDrive;
    }

    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        if (iter == 0) {
            swerveDrive.zeroHeading();
            iter++;
        }    
        timer.start();
            // iter += 1;
        // }

        // System.out.println("WAITING.....");
        // for (int i = 0; i < 10; i++) {
            // System.out.println(timer.get());
        // }
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
        // for (int i = 0; i < 100; i++) {
        //     System.out.println("END END END");
        // }
    }

}
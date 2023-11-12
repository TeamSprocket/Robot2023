package frc.robot.commands.macro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class AlignConeScore extends CommandBase{
    double xSpeed;
    double tSpeed = 0.0;
    public enum Direction {FRONT};
    SwerveDrive swerveDrive;
    public AlignConeScore(double xSpeed, SwerveDrive swerveDrive){
        this.xSpeed = xSpeed/5.0;
        this.swerveDrive = swerveDrive;
    }

    public void initialize() {
      
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double yValue = ty.getDouble(0.0);
        if(yValue > 0){
            swerveDrive.setModuleSpeeds(xSpeed, -0.1, tSpeed);
        } else if(yValue < 0){
            swerveDrive.setModuleSpeeds(xSpeed, 0.1, tSpeed);
        }
        swerveDrive.setHeadingRad(Math.PI);
    }
    
    public void end(){
        swerveDrive.stopModules();
    }
}

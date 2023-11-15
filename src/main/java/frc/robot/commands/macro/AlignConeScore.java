package frc.robot.commands.macro;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class AlignConeScore extends CommandBase{
    double xSpeed;
    double tSpeed = 0.0;
    SwerveDrive swerveDrive;
    Alliance alliance;
    double yValue;

    public enum Sides {RED, BLUE, NONE};
    // private final SendableChooser<String> sideChooser = new SendableChooser<>();

    public AlignConeScore(double xSpeed, SwerveDrive swerveDrive, Alliance alliance){
        this.xSpeed = xSpeed/5.0;
        this.swerveDrive = swerveDrive;
        this.alliance = alliance;

    }

    public void initialize() {
      
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        if (alliance == Alliance.Red) {
            NetworkTableEntry ty = table.getEntry("tyRed");
            double yValue = ty.getDouble(0.0);
        } else if (alliance == Alliance.Blue) {
            NetworkTableEntry ty = table.getEntry("tyBlue");
            double yValue = ty.getDouble(0.0);
        } else if (alliance == Alliance.Invalid){
            yValue = 0.00;
        }

        if(yValue > 0.00){
            swerveDrive.setModuleSpeeds(xSpeed, -0.1, tSpeed);
        } else if(yValue < 0.00){
            swerveDrive.setModuleSpeeds(xSpeed, 0.1, tSpeed);
        }
        swerveDrive.setHeadingRad(Math.PI);
    }
    
    public void end(){
        swerveDrive.stopModules();
    }
}

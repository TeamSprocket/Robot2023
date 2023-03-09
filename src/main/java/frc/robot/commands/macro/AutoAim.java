package frc.robot.commands.macro;

import frc.util.commands.MacroCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.commands.*;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDrive;


public class AutoAim extends MacroCommand{
    

    private final Limelight limelight;
    private final SwerveDrive swerveDrive;
    double xOffset;   
    double yOffset;
    double aOffset;
    private final PIDController controller;

    public AutoAim(Limelight limelight, SwerveDrive swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.controller = new PIDController(Constants.Auton.kPBalance, Constants.Auton.kIBalance, Constants.Auton.kDBalance);

    }
    @Override
    public void initialize(){
        limelight.findDistance();
    }

    @Override
    public void execute(){
        double xGoHere = 0;
        double yGoHere = 0;
        xOffset = limelight.getTx();     //target 6.22
        yOffset = limelight.getTy();     //target -7.72
        aOffset = limelight.getTa();     //target 0.047

        if (xOffset > 6.22) { //other way around?
            xGoHere = 0.3;
        }
        else {
            xGoHere = -0.3;
        }
        double xOutput = controller.calculate(swerveDrive.getDrivePosition(), xGoHere);
        if (xOutput >= 0.1) {
          xOutput = 0.1;
        }

        if (yOffset > -7.72) { //other way around?
            yGoHere = 0.3;
        }
        else {
            yGoHere = -0.3;
        }
        double yOutput = controller.calculate(swerveDrive.getDrivePosition(), yGoHere);
        if (yOutput >= 0.1) {
          yOutput = 0.1;
        }
        
        //wtf does this mean bro
        ChassisSpeeds chassisSpeeds;
        if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
          double headingRad = Math.toRadians(swerveDrive.getHeading());
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xOutput, yOutput, 0, new Rotation2d(headingRad));
        } else { 
          chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, 0);
        }
    
        // Calculate module states per module
        SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
      
        // Apply to modules
        swerveDrive.setModuleStates(moduleStates);

    }

    @Override
    public boolean isFinished(){
        if (limelight.getTx() > 6.15 && limelight.getTx() < 6.3 && limelight.getTy() < -7.6 && limelight.getTy() > -7.9) {
            return true;
        }
        return false;
    }

    @Override 
    public void end(boolean interrupted){
    }

  
}

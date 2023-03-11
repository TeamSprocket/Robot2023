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
        //limelight.findDistance();
    }

    @Override
    public void execute(){
        double xGoHere = 0;
        double yGoHere = 0;
        double aGoHere = 0;
        xOffset = limelight.getTx();     //target 6.22
        yOffset = limelight.getTy();     //target -7.72
        aOffset = limelight.getTa();     //target 0.047

        //MOVE X AXIS
        if (xOffset > 6.3) { //other way around?
            xGoHere = 0.3;
        }
        else if (xOffset < 6.15) {
            xGoHere = -0.3;
        }
        else {
            xGoHere = 0;
        }
        double xOutput = controller.calculate(swerveDrive.getDrivePosition(), xGoHere);
        if (xOutput >= 0.1) {
          xOutput = 0.1;
        }

        //MOVE Y AXIS
        if (yOffset > -7.85) { //other way around?
            yGoHere = 0.3;
        }
        else if (yOffset < -7.6) {
            yGoHere = -0.3;
        }
        else {
            yGoHere = 0;
        }
        double yOutput = controller.calculate(swerveDrive.getDrivePosition(), yGoHere);
        if (yOutput >= 0.1) {
          yOutput = 0.1;
        }
        
        //MOVE BY ROTATION
        if (aOffset > 0.06) { //other way around?
            aGoHere = 0.1;
        }
        else if (aOffset < 0.035) {
            aGoHere = -0.1;
        }
        else {
            aGoHere = 0;
        }
        double aOutput = controller.calculate(swerveDrive.getHeading(), aGoHere);
        if (aOutput >= 0.1) {
          aOutput = 0.1;
        }

        //wtf does this mean bro
        ChassisSpeeds chassisSpeeds;
        if (Constants.Drivetrain.IS_FIELD_ORIENTED) {
          double headingRad = Math.toRadians(swerveDrive.getHeading());
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xOutput, yOutput, aOutput, new Rotation2d(headingRad));
        } else { 
          chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, aOutput);
        }
    
        // Calculate module states per module
        SwerveModuleState[] moduleStates = Constants.Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
      
        // Apply to modules
        swerveDrive.setModuleStates(moduleStates);

    }

    @Override
    public boolean isFinished(){
        if (xOffset > 6.15 && xOffset < 6.3 && yOffset < -7.6 && yOffset > -7.85 && aOffset > 0.035 && aOffset < 0.06) {
            return true;
        }
        return false;
    }

    @Override 
    public void end(boolean interrupted){
    }

  
}

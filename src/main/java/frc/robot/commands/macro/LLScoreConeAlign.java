
package frc.robot.commands.macro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LLShootCone;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LLScoreConeAlign extends CommandBase {
  SwerveDrive swerveDrive;
  Claw claw;
  Alliance alliance;
  PIDController pidController = new PIDController(Constants.LL.kPScoreConeAlignY, Constants.LL.kIScoreConeAlignY, Constants.LL.kDScoreConeAlignY);

  public LLScoreConeAlign(SwerveDrive swerveDrive, Claw claw, Alliance alliance) {
    this.swerveDrive = swerveDrive;
    this.claw = claw;
    this.alliance = alliance;

    pidController.setSetpoint(Constants.LL.kYPosSetpoint);
  }

  @Override
  public void initialize() {
    swerveDrive.setTargetHeading(0.0);
    Constants.isScoreConeAlign = true;
  }


  @Override
  public void execute() {
    double yPos;
  
    if (alliance.equals(Alliance.Blue)) {
      yPos = NetworkTableInstance.getDefault().getTable("limelight")
          .getEntry("botpose_wpiblue").getDoubleArray(new double[6])[0];
    } else {
      yPos = NetworkTableInstance.getDefault().getTable("limelight")
          .getEntry("botpose_wpired").getDoubleArray(new double[6])[0];
    }

    swerveDrive.setScoreConeAlignSpeed(pidController.calculate(yPos));
  }


  
  @Override
  public void end(boolean interrupted) {
    new LLShootCone(claw);
    Constants.isScoreConeAlign = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

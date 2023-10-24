
package frc.robot.commands.macro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class LLCubeAlign extends CommandBase {
  SwerveDrive swerveDrive;
  PIDController pidController = new PIDController(Constants.Drivetrain.kLimelightAlignP, Constants.Drivetrain.kLimelightAlignI, Constants.Drivetrain.kLimelightAlignD);
  Timer timer;
  double duration;

  public LLCubeAlign(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.duration = 0;
    pidController.setSetpoint(0);
  } 
  public LLCubeAlign(SwerveDrive swerveDrive, double duration) {
    this.swerveDrive = swerveDrive;
    this.duration = duration;
    pidController.setSetpoint(0);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    timer.start();

    double cubeX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // -27 to 27 deg
    double vX = pidController.calculate(cubeX);
    swerveDrive.setModuleSpeedsHeading(vX, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (duration == 0) {
      return timer.get() <= duration;
    } 
    return false;
  }
}

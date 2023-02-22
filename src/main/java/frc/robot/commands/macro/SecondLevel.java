
package frc.robot.commands.macro;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.commands.MacroCommand;
public class SecondLevel extends MacroCommand {

  private final Elevator elevator;
  private final Timer timer;
  
  public SecondLevel(Elevator elevator) {
    this.elevator = elevator;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    timer.start();
    new ElevatePosition(elevator, 18.6);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stuff
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new ElevatePosition(elevator, 34.45866374);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= 2) {
      return true;
    }
    return false;
  }
}

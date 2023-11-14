

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class LLShootCone extends CommandBase {
  Timer timer = new Timer();
  Claw claw;

  public LLShootCone(Claw claw) {
    this.claw = claw;
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    timer.start();
    claw.moveClaw(Constants.LL.kShootConeSpeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.get() <= Constants.LL.kShootConeDuration;
  }
}

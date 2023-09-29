package frc.robot.commands.persistent;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class ElevateTimed extends CommandBase {
    private final Elevator elevator;
    private final Timer timer;
    private final double duration, speed;
    // private final Elevate elevate;

  
    public ElevateTimed (Elevator elevator, double speed, double duration) {
      this.elevator = elevator;
      this.speed = speed;

      this.duration = duration;

      timer = new Timer();
  
      addRequirements(elevator);
    }
  
    @Override
    public void initialize() {
      timer.reset();
      // new Elevate(elevator, speed);
    }

    @Override
    public void execute() {
      timer.start();
    }

    @Override
    public boolean isFinished() {
      if (timer.get() >= duration) {
        return true;
      }
      return false;
    }
  
    @Override
    public void end(boolean interrupted) {
        // elevate.stop();
        elevator.stop();
    }
}

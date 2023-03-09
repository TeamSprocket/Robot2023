package frc.robot.commands.macro.timed
;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.util.Util;
// import frc.util.commands.PersistentCommand;

public class RollClawTimed extends CommandBase {
    private final Claw claw;
    private final double duration, speed;
    private final Timer timer;

    /**
     * 
     * @param claw claw obnject
     * @param duration length in seconds the auton will run 
     */
    public RollClawTimed(Claw claw, double duration) {
        this.claw = claw;
        this.speed = 1.0;
        this.duration = duration;

        this.timer = new Timer();

        addRequirements(claw);
    }

    /**
     * 
     * @param claw claw obnject
     * @param speed speed from -1 to 1 to run the motor
     * @param duration length in seconds the auton will run 
     */
    public RollClawTimed(Claw claw, double speed, double duration) {
      this.claw = claw;
      this.speed = speed;
      this.duration = duration;

      this.timer = new Timer();

      addRequirements(claw);
  }

    @Override
    public void initialize() {
      timer.reset();
    }

    @Override
    public void execute() { 
      timer.start();
      claw.moveClaw(-speed);

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
        claw.moveClaw(0);
    }

}
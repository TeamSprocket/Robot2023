package frc.robot.commands.macro.timed
;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.util.Util;
// import frc.util.commands.PersistentCommand;

public class RollIntakeTimed extends CommandBase {
    private final Intake intake;
    private final double duration, speed;
    private final Timer timer;

    /**
     * 
     * @param intake intake obnject
     * @param duration length in seconds the auton will run 
     */
    public RollIntakeTimed(Intake intake, double duration) {
        this.intake = intake;
        this.speed = 1.0;
        this.duration = duration;

        this.timer = new Timer();

        addRequirements(intake);
    }

    /**
     * 
     * @param intake intake obnject
     * @param speed speed from -1 to 1 to run the motor
     * @param duration length in seconds the auton will run 
     */
    public RollIntakeTimed(Intake intake, double speed, double duration) {
      this.intake = intake;
      this.speed = speed;
      this.duration = duration;

      this.timer = new Timer();

      addRequirements(intake);
  }

    @Override
    public void initialize() {
      timer.reset();
    }

    @Override
    public void execute() { 
      timer.start();
      intake.moveIntake(-speed);

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
        intake.moveIntake(0);
    }

}
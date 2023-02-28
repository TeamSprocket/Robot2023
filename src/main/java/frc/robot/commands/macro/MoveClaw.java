package frc.robot.commands.macro;
import edu.wpi.first.wpilibj.XboxController;
import frc.util.Util;
import frc.util.commands.MacroCommand;
import frc.robot.subsystems.Wheels;


public class MoveClaw extends MacroCommand {
    private final Wheels clawMovement;

    private final XboxController driveController;

    public MoveClaw(Wheels clawMovement, XboxController driveController) {
        this.clawMovement = clawMovement;
        this.driveController = driveController;
    }

    @Override
    public void execute() {
        double input = driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis(); 
        double deadbandedInput = Util.deadband(0.1, input);

        clawMovement.setSpeed(0.10 * deadbandedInput);

    }
    @Override
    public void end(boolean interrupted) {
        clawMovement.setSpeed(0);
    }

    @Override
    public void initialize() {

    }



    @Override
    public boolean isFinished () {
        return false;
    }

}
package frc.robot.commands.macro;
import edu.wpi.first.wpilibj.XboxController;
import frc.util.Util;
import frc.util.commands.MacroCommand;
import frc.robot.subsystems.Wheels;


public class MoveWheels extends MacroCommand {
    private final Wheels wheelMovement;

    private final XboxController driveController;

    public MoveWheels(Wheels wheelMovement, XboxController driveController) {
        this.wheelMovement = wheelMovement;
        this.driveController = driveController;
    }

    @Override
    public void execute() {
        double input = driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis(); 
        double deadbandedInput = Util.deadband(0.1, input);

        wheelMovement.setOutput(0.10 * deadbandedInput);

    }
    @Override
    public void end(boolean interrupted) {
        wheelMovement.setOutput(0);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished () {
        return false;
    }

}
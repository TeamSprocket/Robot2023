package frc.robot.commands.persistent
;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class RollClaw extends PersistentCommand {
    private final Claw claw;
    private final XboxController controller;

    public RollClaw(Claw claw, XboxController controller) {
        this.claw = claw;
        this.controller = controller;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        double input = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(); 
        double deadbandedInput = Util.deadband(0.1, input);

        claw.moveClaw(deadbandedInput*0.3);

    }
    @Override
    public void end(boolean interrupted) {
        claw.moveClaw(0);
    }

}
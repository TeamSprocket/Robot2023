package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class ElevateManual extends PersistentCommand {
    private final Elevator elevator;
    private final XboxController gamepad;
  
    public ElevateManual (Elevator elevator, XboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
  
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
      double input = gamepad.getRightY();
      double deadbandedInput = Util.deadband(0.1, input);
      elevator.setOutputManual(deadbandedInput);
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

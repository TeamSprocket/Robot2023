package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class SetElevatorBase extends InstantCommand {
    private final Elevator elevator;
    private final XboxController gamepad;
    double desiredHeightInMeters = 1.063625;
  
    public SetElevatorBase (Elevator elevator, XboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
  
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
      elevator.setElevatorEncoderBase(elevator.getElevtorHeightInMeters());
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

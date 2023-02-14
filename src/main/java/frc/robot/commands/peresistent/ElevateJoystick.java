package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class ElevateJoystick extends PersistentCommand {
    private final Elevator elevator;
    private final XboxController gamepad;
  
    public ElevateJoystick (Elevator elevator, XboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
  
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
      double leftY = gamepad.getLeftY();
      double deadbandedInput = Util.deadband(0.1, leftY);

      // elevator.setOutputManual(deadbandedInput);
      double targetHeightMeters = (deadbandedInput + 1.0) / 2.0 * Constants.Elevator.MAX_HEIGHT_METERS;
      elevator.setElevatorHeight(targetHeightMeters);

      // if (deadbandedInput > 0 && elevator.getElevtorHeightInMeters() > Constants.Elevator.MAX_HEIGHT_METERS){
      //   deadbandedInput = 0;
      // }
      // else if (deadbandedInput < 0 && elevator.getElevtorHeightInMeters() < Constants.Elevator.MIN_HEIGHT_METERS){
      //   deadbandedInput = 0;
      // }
      // else{
      //   elevator.setOutputManual(deadbandedInput);
      // }
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

      elevator.moveElevator(deadbandedInput);

    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    // @Override
    // public void periodic(){
    //     SmartDashboard.putNumber("Elvator Height", getElevtorHeightInMeters);

    // }
}

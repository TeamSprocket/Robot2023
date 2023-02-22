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
      System.out.println("DEADBAND INPUT: " + leftY);
      System.out.println("HEIGHT: " + elevator.getElevatorHeight());

      if (deadbandedInput == 0) {
        double output = 0;
        if (elevator.getElevatorHeight() > 27)
          output += 0.005;
        
        output += (-0.0000212*elevator.getElevatorHeight()-0.0532628);
        elevator.moveElevator(output);
        
      }
      else{
        elevator.moveElevator(deadbandedInput);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

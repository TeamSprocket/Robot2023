package frc.robot.commands.persistent;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class Elevate extends PersistentCommand {
    private final Elevator elevator;
    private final XboxController gamepad;

  
    public Elevate (Elevator elevator, XboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
  
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {

      double leftY = gamepad.getLeftTriggerAxis() - gamepad.getRightTriggerAxis(); 

      // double leftY = gamepad.getLeftY();

      double deadbandedInput = Util.deadband(0.1, leftY);
    

      if (deadbandedInput == 0) {
        double output = 0;
        // System.out.println(elevator.getElevatorHeight());
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

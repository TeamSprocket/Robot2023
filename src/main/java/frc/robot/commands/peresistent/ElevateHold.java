package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class ElevateHold extends PersistentCommand {
    private final Elevator elevator;
    private final XboxController gamepad;
    private double targetHeight;
  
    public ElevateHold (Elevator elevator, XboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
  
      targetHeight = ElevateJoystick.getTargetHeight();

      addRequirements(elevator);
    }
  
    

    @Override
    public void execute() {

      /////////////////////////////////////////////////////////////
        elevator.setElevatorHeight(targetHeight);

      /////////////////////////////////////////////////////////
      //USING ENCODER
      // double targetSetpoint = (deadbandedInput + 1.0) / 2.0 * Constants.Elevator.ENCODER_RANGE;
      // if (deadbandedInput > 0 && elevator.getElevtorHeightInMeters() > Constants.Elevator.MAX_ENCODER_VALUE){
      //   deadbandedInput = 0;
      // }
      // else if (deadbandedInput < 0 && elevator.getElevtorHeightInMeters() < Constants.Elevator.MIN_ENCODER_VALUE){
      //   deadbandedInput = 0;
      // }
      // else{
      //   if (direction ==-1){
      //     elevator.setElevatorHeightEncoder(-targetSetpoint);
      //   }
      //   else{
      //     elevator.setElevatorHeightEncoder(targetSetpoint);
      //   }
      // }

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

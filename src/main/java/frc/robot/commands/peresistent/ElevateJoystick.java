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

    private static double elevatorOutput;
    private static double targetHeight;
  
    public ElevateJoystick (Elevator elevator, XboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
  
      addRequirements(elevator);
    }

    public static double getTargetHeight(){
      return targetHeight;
    }

    public static double getElevatorOutput(){
      return elevatorOutput;
    }
  
    @Override
    public void execute() {
      double leftY = gamepad.getLeftY();

      double deadbandedInput = Util.deadband(0.1, leftY);

      /////////////////////////////////////////////////////////////
      //USING HEIGHT
      double targetHeightMeters = (deadbandedInput) * Constants.Elevator.MAX_HEIGHT_METERS;
      targetHeight = targetHeightMeters;

      if (deadbandedInput == 0){
        double lEValue = elevator.getLeftEncoder();
        double tHM;
        //lEValue = 4.133*(tHM^2) - 0.061*(tHM) - 0.102;
        //tHM = (0.061 + Math.sqrt(0.061*0.061 - 4*4.133*-0.102) ) / (2*(4.133)) - lEValue;
        tHM = Math.sqrt((lEValue + 0.127)/4.109);
        System.out.println(tHM);
        // if(lEValue > Constants.Elevator.MAX_ENCODER_VALUE){
        //   lEValue = 23;
        // }
        // else if (lEValue < Constants.Elevator.MIN_ENCODER_VALUE){
        //   lEValue = 0;
        // }
        //else{
          if (lEValue > 6 && lEValue < 14){
            tHM -= 0.05;
          }
          if (lEValue > 20) {
            tHM += 0.05;
          }    
        //}
        elevator.setElevatorHeight(tHM-0.05);
      }

      


      else if (deadbandedInput > 0 && elevator.getElevtorHeightInMeters() > Constants.Elevator.MAX_HEIGHT_METERS){
        deadbandedInput = 0;
      }
      else if (deadbandedInput < 0 && elevator.getElevtorHeightInMeters() < Constants.Elevator.MIN_HEIGHT_METERS){
        deadbandedInput = 0;
      }
      else{
        elevator.setElevatorHeight(targetHeightMeters);
      }

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

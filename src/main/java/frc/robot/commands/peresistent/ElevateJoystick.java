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

      if (deadbandedInput == 0) {
        double lEValue = elevator.getLeftEncoder() + 16.2;  //range 0-23
        double tHM; //output speed
        //lEValue = 4.133*(tHM^2) - 0.061*(tHM) - 0.102;
        //tHM = (0.061 + Math.sqrt(0.061*0.061 - 4*4.133*-0.102) ) / (2*(4.133)) - lEValue;
        tHM = Math.sqrt((lEValue + 0.127)/4.109); //math
        System.out.println(lEValue); //check

        //CHECK MAX/MIN VALUES
        // if(lEValue > Constants.Elevator.MAX_ENCODER_VALUE){
        //   lEValue = 23;
        // }
        // if (lEValue < Constants.Elevator.MIN_ENCODER_VALUE){
        //   lEValue = 0;
        // }


        if (lEValue > 6 && lEValue < 17){ //compensate error
          tHM -= 0.5;
        }
        if (lEValue > 17 && lEValue < 20){ //compensate error
          tHM -= 0.1;
        }
        if (lEValue > 20 && lEValue < 22.5) { //compensate error
          tHM -= 0.05;  
        }    
        elevator.setElevatorHeight(tHM - 0.75); //set speed + compensate error
      }


      // else if (deadbandedInput > 0 && elevator.getElevtorHeightInMeters() > Constants.Elevator.MAX_HEIGHT_METERS){
      //   deadbandedInput = 0;
      // }
      // else if (deadbandedInput < 0 && elevator.getElevtorHeightInMeters() < Constants.Elevator.MIN_HEIGHT_METERS){
      //   deadbandedInput = 0;
      // }
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

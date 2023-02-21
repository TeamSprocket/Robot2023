package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class MoveArmJoystick extends PersistentCommand {
    private final Arm arm;
    // private final Elevator elevator;
    private final XboxController gamepad;
  
    public MoveArmJoystick (Arm arm, XboxController gamepad) {
        this.arm = arm;
        // this.elevator = elevator;
        this.gamepad = gamepad;
  
        addRequirements(arm);
    }
  
    @Override
    public void execute() {

      // double armMaxHeight;
      // double armMinHeight;
      // double armRange = Constants.Arm.MAX_ENCODER - Constants.Arm.MIN_ENCODER_ELEVATOR_DOWN;

      // double elevatorHeight = elevator.getElevtorHeight();
      // if (elevatorHeight >= 0){
      //   armMaxHeight = Constants.Arm.MAX_HEIGHT_METERS_ELEVATOR;
      //   armMinHeight = Constants.Arm.MIN_HEIGHT_METERS_ELEVATOR;
      //   armRange = Constants.Arm.HEIGHT_METERS_ELEVATOR;
      // } 
      // else{
      //   armMaxHeight = Constants.Arm.MAX_HEIGHT_METERS_NO_ELEVATOR;
      //   armMinHeight = Constants.Arm.MIN_HEIGHT_METERS_NO_ELEVATOR;
      //   armRange = Constants.Arm.HEIGHT_METERS_NO_ELEVATOR;
      // }

      double rightY = gamepad.getRightY();
      double armInput = Util.deadband(0.1, rightY);

      double targetArmPosition = (armInput) * (Constants.Arm.MAX_ENCODER - Constants.Arm.MIN_ENCODER_ELEVATOR_DOWN);
      if (arm.getArmLeftEncoder() == Constants.Arm.MAX_ENCODER){
        arm.setArmPosition(Constants.Arm.MAX_ENCODER - Constants.Arm.MIN_ENCODER_ELEVATOR_DOWN);
      }
      else{
        arm.setArmPosition(targetArmPosition);

      }
      /*
      if (deadbandedInput == 0) {
        double lEValue = elevator.getLeftEncoder() + 16.2;  //range 0-23
        double tHM; //output speed
        //lEValue = 4.133*(tHM^2) - 0.061*(tHM) - 0.102;
        //tHM = (0.061 + Math.sqrt(0.061*0.061 - 4*4.133*-0.102) ) / (2*(4.133)) - lEValue;
        tHM = Math.sqrt((lEValue + 0.127)/4.109); //math
        System.out.println(lEValue); //check
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
        */

      // if (armInput == 0){
      //   double armValue = arm.getArmLeftEncoder();
      //   double tFM;
      //   //tFM = (0.061 + Math.sqrt(0.061*0.061 - 4*4.133*-0.102) ) / (2*(4.133)) - armValue;
      //   // tFM = Math.sqrt((armValue + 0.127)/4.109); //math
      //   // System.out.println(armValue); //check
      // }

      //PID VERSION - adjust PID values

      //AFF version
      // arm.setArmPositionAFF(targetHeight);
        
    }
  
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}

// package frc.robot.commands.peresistent;

// import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.Constants;
// import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Arm;
// import frc.util.Util;
// import frc.util.commands.PersistentCommand;

// public class MoveWristManual extends PersistentCommand {
//     private final Wrist wrist;
//     // private final Arm arm;
//     private final XboxController gamepad;
  
//     public MoveWristManual (Wrist wrist, XboxController gamepad) {
//         this.wrist = wrist;
//         // this.arm = arm;
//         this.gamepad = gamepad;
  
//         addRequirements(wrist);
//     }
  
//     @Override
//     public void execute() {

//       double wristMaxHeight;
//       double wristMinHeight;
//       // double wristRange = Constants.Wrist.ENCODER_RANGE;

//       double input = gamepad.getLeftTriggerAxis() - gamepad.getRightTriggerAxis(); 
// 		  double deadbandedInput = Util.deadband(0.1, input);

//       // double armPosition = arm.getArmPosition();
//       // if (armPosition > 0){
//       //   wristMaxHeight = Constants.Arm.MAX_HEIGHT_METERS_ELEVATOR;
//       //   wristMinHeight = Constants.Arm.MIN_HEIGHT_METERS_ELEVATOR;
//       //   wristRange = Constants.Arm.HEIGHT_METERS_ELEVATOR;
//       // }
//       // else{
//       //   wristMaxHeight = Constants.Arm.MAX_HEIGHT_METERS_NO_ELEVATOR;
//       //   wristMinHeight = Constants.Arm.MIN_HEIGHT_METERS_NO_ELEVATOR;
//       //   wristRange = Constants.Arm.HEIGHT_METERS_NO_ELEVATOR;
//       // }

//       // double targetHeight = (deadbandedInput) * wristRange;

//       /*
//       if (deadbandedInput == 0) {
//         double lEValue = elevator.getLeftEncoder() + 16.2;  //range 0-23
//         double tHM; //output speed
//         //lEValue = 4.133*(tHM^2) - 0.061*(tHM) - 0.102;
//         //tHM = (0.061 + Math.sqrt(0.061*0.061 - 4*4.133*-0.102) ) / (2*(4.133)) - lEValue;
//         tHM = Math.sqrt((lEValue + 0.127)/4.109); //math
//         System.out.println(lEValue); //check
//         if (lEValue > 6 && lEValue < 17){ //compensate error
//           tHM -= 0.5;
//         }
//         if (lEValue > 17 && lEValue < 20){ //compensate error
//           tHM -= 0.1;
//         }
//         if (lEValue > 20 && lEValue < 22.5) { //compensate error
//           tHM -= 0.05;  
//         }    
//         elevator.setElevatorHeight(tHM - 0.75); //set speed + compensate error
//         */

//       //PID VERSION - adjust PID values
//       wrist.setWrist(targetHeight);

//       //AFF version
//       // arm.setArmPositionAFF(targetHeight);
        
//     }
  
//     @Override
//     public void end(boolean interrupted) {
//       wrist.stop();
//     }
// }

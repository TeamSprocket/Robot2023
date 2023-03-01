// package frc.robot.commands.macro;

// import frc.robot.subsystems.Elevator;
// import frc.util.commands.MacroCommand;
// import edu.wpi.first.wpilibj.Timer;


// public class ElevatePositionTimed extends MacroCommand {
//     private final Elevator elevator;
//     private final double seconds;
//     private final Timer timer;

//     private double startTime;
  
//     public ElevatePositionTimed (Elevator elevator, double seconds) {
//       this.elevator = elevator;
//       this.seconds = seconds;
//       timer = new Timer();
  
//       addRequirements(elevator);
//     }

//     public void initialize(){
//       startTime = System.currentTimeMillis();
//     }

//     @Override
//     public void execute() {
//       timer.start();
//       elevator.setElevatorPosition(elevator.getElevatorHeight(), height);
//     }

//     public boolean isFinished(){
//         // if (timer.get() >= seconds) {
//         //     return true;
//         // }
//         // else {
//             return false;
//         // }
//     }

//     @Override
//     public void end(boolean interrupted){
//         elevator.moveElevator(height);
//     }
// }

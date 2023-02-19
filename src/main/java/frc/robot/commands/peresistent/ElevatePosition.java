package frc.robot.commands.peresistent;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class ElevatePosition extends PersistentCommand {
    private final Elevator elevator;
    private final double encoder;
  
    public ElevatePosition (Elevator elevator, double encoder) {
      this.elevator = elevator;
      this.encoder = encoder;
  
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {

      //add offset from starting elevator higher
      double lEValue = encoder + 16.46;
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
      if (lEValue > 6 && lEValue < 14){
        tHM -= 0.05;
      }
      if (lEValue > 20) {
        tHM += 0.05;
      }    
      elevator.setElevatorHeight(tHM-0.05);
    }
  
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

package frc.robot.commands.macro;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.util.Util;
import frc.util.commands.MacroCommand;
import frc.util.commands.PersistentCommand;

public class ElevatePosition extends MacroCommand {
    private final Elevator elevator;
    private final double encoder;
  
    public ElevatePosition (Elevator elevator, double encoder) {
      this.elevator = elevator;
      this.encoder = encoder;
  
      addRequirements(elevator);
    }

    public void initialize(){
      
    }

    @Override
    public void execute() {

      //add offset from starting elevator higher
      double lEValue = encoder + 16.2;
      double tHM;
      //lEValue = 4.133*(tHM^2) - 0.061*(tHM) - 0.102;
      //tHM = (0.061 + Math.sqrt(0.061*0.061 - 4*4.133*-0.102) ) / (2*(4.133)) - lEValue;
      tHM = Math.sqrt((lEValue + 0.127)/4.109);
      // System.out.println("Elevator Output" + tHM);
      // if(lEValue > Constants.Elevator.MAX_ENCODER_VALUE){
      //   lEValue = 23;
      // }
      // else if (lEValue < Constants.Elevator.MIN_ENCODER_VALUE){
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
  
    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
    }
}

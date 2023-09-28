package frc.robot.commands.persistent;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class MoveWristManual extends PersistentCommand {
    private final Wrist wrist;
    private final CommandXboxController gamepad;
  
    public MoveWristManual (Wrist wrist, CommandXboxController gamepad) {
        this.wrist = wrist;
        this.gamepad = gamepad;
  
        addRequirements(wrist);
    }
  
    @Override
    public void execute() {
      // pos range 6.7 bottom to -3.8 top
      double input = gamepad.getLeftY();
    
      double wristInput = Util.deadband(0.1, input);
    //   System.out.println("ANGLE: " + wrist.getWristAngle() + "WRIST INPUT: " + wristInput);
      //System.out.println("WRIST ANGLE: " + wrist.getWristAngle());
    //   if (wristInput == 0) {
    //     double output = wristInput;

    //     if (wrist.getWristAngle() == 0) {
    //         // -= goes down
    //         // += goes up
    //         output = 0.00379241 * wrist.getWristAngle() - 0.492261;

    //         //44.57   -0.289
    //        //25.247  -0.375
    //        //5.298   -0.469
    //         //62.649    -0.25
    //         //37.403 -0.414

    //         if (wrist.getWristAngle() < 50 && wrist.getWristAngle() > 0) {
    //             output -= 0.05;
    //             if (wrist.getWristAngle() < 40) {
    //                 output -= 0.05;
    //             }
    //         }
    //         if (wrist.getWristAngle() < 0) {
    //             output += 0.05;
    //         }
    //         wrist.moveWrist(output);
    //     }

        
    //     }
    // else {
    // }
    // wrist.moveWrist(wristInput * 0.2);

    double wristOutput = (wristInput * 1.0) + wrist.getWristAngle(); // TODO: TUNE
    wrist.setWristAngle(wristOutput);


    }

  
    @Override
    public void end(boolean interrupted) {
        wrist.stop();

    }
}

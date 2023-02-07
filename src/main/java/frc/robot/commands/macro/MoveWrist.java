package frc.robot.commands.macro;
import edu.wpi.first.wpilibj.XboxController;
import frc.util.Util;
import frc.util.commands.MacroCommand;
import frc.robot.subsystems.Wrist;
//import frc.robot.RobotMap;

public class MoveWrist extends MacroCommand {
    private final Wrist wristMovement;
    
    private final XboxController gamepad;

    public MoveWrist(Wrist wristMovement, XboxController gamepad) {
        this.wristMovement = wristMovement;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        
        double input = gamepad.getRightY();
        double deadbandedInput = Util.deadband(0.1, input);
        
        wristMovement.setOutput(0.10*deadbandedInput);
    }

    @Override
    public void end(boolean interrupted){
        wristMovement.setOutput(0);

}

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}

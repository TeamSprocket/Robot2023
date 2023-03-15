package frc.robot.commands.persistent
;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.util.Util;
import frc.util.commands.PersistentCommand;

public class RollClaw extends CommandBase {
    private final Claw claw;
    private final XboxController controller;
    // double input;
    // private final Supplier<Double> inputFunct;

    public RollClaw(Claw claw, XboxController controller) {
        this.claw = claw;
        this.controller = controller;
        // this.input = inputFunct.get();
        // this.input = 0.1;

        addRequirements(claw);
    }

    @Override
    public void execute() { 
        double input = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        double deadbandcedInput = Util.deadband(0.1, input);
        input /= 3.0;
        
        // if (input > 1.0) {
        //     input = 1;
        // }
        // if (input < -1.0) {
        //     input = -1;
        // }

        claw.moveClaw(deadbandcedInput);
        // claw.moveClaw(0.1);

        // System.out.println(deadbandedInput);
        // SmartDashboard.putNumber("Intake Speed", input);
        

    }
    @Override
    public void end(boolean interrupted) {
        claw.moveClaw(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
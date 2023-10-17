package frc.robot.commands.superstructure
;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.util.Util;
import frc.util.wpilib_defaults.commands.PersistentCommand;

public class SetMidCube extends CommandBase {
    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer timer;
    // double input;
    // private final Supplier<Double> inputFunct;

    public SetMidCube(Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;

        timer = new Timer();

        addRequirements(elevator, arm, wrist);

        // Constants.Drivetrain.CAN_DIRECTION_SWITCH = false;
    }

    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        
        if (timer.get() > 0 && timer.get() < 1){
            wrist.setWristAngle(wrist.getWristAngle(), -22.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -63, 0.4);
            elevator.setElevatorPositionSpeed(0, 5, 0.6);
        }
        else{   
            wrist.setWristAngle(wrist.getWristAngle(), -22.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -63, 0.2);
            elevator.setElevatorPositionSpeed(0, 5, 0.2);
        }
        
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted){
        wrist.setWristAngle(wrist.getWristAngle(), -22.5);
            arm.setArmAngleSpeed(arm.getArmAngle(), -63, 0.2);
            elevator.setElevatorPositionSpeed(0, 5, 0.2);
    }

}
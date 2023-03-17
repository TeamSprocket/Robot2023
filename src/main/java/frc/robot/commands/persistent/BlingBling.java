package frc.robot.commands.persistent;

import frc.robot.Constants;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Elevator;
import frc.util.commands.PersistentCommand;


public class BlingBling extends PersistentCommand {
    private final LEDStrip lights;
    private final Elevator elevator;

    public BlingBling(LEDStrip lights, Elevator elevator) {
      this.lights = lights;
      this.elevator = elevator;

      addRequirements(lights);
    }

    @Override
    public void execute() {
        double height = elevator.getElevatorHeight();
        
        if (height <= 1) {  // elevator height good
            lights.strobeWhite();
        }
        else {
            lights.heartBeatWhite();
        }
    }

    @Override
    public void end(boolean interrupted) {
        lights.set(0);
    }
}
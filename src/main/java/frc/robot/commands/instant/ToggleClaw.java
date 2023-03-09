package frc.robot.commands.instant;

import frc.robot.subsystems.Claw;
import frc.util.commands.InstantCommand;

public class ToggleClaw extends InstantCommand{
    private final Claw claw;

    public ToggleClaw(Claw claw){
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.actuateClaw(claw.isActuated());
    }
}
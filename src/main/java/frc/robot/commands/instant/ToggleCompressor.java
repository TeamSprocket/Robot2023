package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PCH;
import frc.util.commands.InstantCommand;

public class ToggleCompressor extends InstantCommand {
    private final PCH pch;
    private final XboxController gamepad;
     
    public ToggleCompressor(PCH pch, XboxController gamepad) {
        this.pch = pch;
        this.gamepad = gamepad;        
        addRequirements(pch);
    }

    @Override
    public void initialize() {
        if(pch.isEnabled()) {
            pch.setCompression(false);
            gamepad.setRumble(RumbleType.kLeftRumble, 0.0);
            SmartDashboard.putString("Compressor Status", "Enabled");
        } else {
            pch.setCompression(true);   
            gamepad.setRumble(RumbleType.kLeftRumble, 1.0);
            SmartDashboard.putString("Compressor Status", "Disabled");
        }
    }   
}

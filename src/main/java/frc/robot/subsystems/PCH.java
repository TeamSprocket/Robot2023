package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class PCH extends SubsystemBase {
    private final PneumaticHub pneumaticHub = new PneumaticHub(RobotMap.PCH.PCH_CAN);
    
    public PCH() {

    }

    public boolean isEnabled() {
        return pneumaticHub.getCompressor();
    }

    public void clearStickyFaults() {
        pneumaticHub.clearStickyFaults();
    }
    
    public void setCompression(boolean on) {
        if(on) {
            pneumaticHub.enableCompressorAnalog(Constants.PCH.MIN_PSI, Constants.PCH.MAX_PSI);
            // pneumaticHub.enableCompressorAnalog(Constants.Compressor.MIN_PSI, Constants.Compressor.MAX_PSI);
        }
        else {
            pneumaticHub.disableCompressor();
        }
    }
    
    public double getPressure() {
        return Math.abs(pneumaticHub.getPressure(RobotMap.PCH.PRESSURE_SENSOR_CHANNEL));
    }
      
    @Override
    public void periodic() {
        
        // if (getPressure() > Constants.PCH.MAX_PSI) {

        // }

        SmartDashboard.putBoolean("[PCM] Compressing", isEnabled());
        SmartDashboard.putNumber("[PCM] Pressure", getPressure());
    }

}
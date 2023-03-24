package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap; 

public class LEDStrip extends SubsystemBase {
    
    private static Spark blink = new Spark(RobotMap.LEDStrip.LED);

    public LEDStrip() {

    }    

    /* 
    values: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    */

    public void set(double val) {
        if ((val >= -1.0) && (val <= 1.0)) {
            blink.set(val);
        }
    }

    public void solidRed() {
        set(0.61);
    }

    public void solidBlueViolet() {
        set(0.89);
    }
    
    public void strobeWhite() {
        set(-0.05);
    }

    public void heartBeatWhite(){
        set(-0.21);
    }
    
    public void solidBlue() {
        set(0.83);
    }

    public String getAllianceColor() {
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return "Blue";
        }
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return "Red";
        }
        return "Nothing";
    }

    public String notAllianceColor() {
        if (getAllianceColor().equals("Blue")){
        return "Red";
        }

        else if (getAllianceColor().equals("Red")) {
        return "Blue";
        }
        
        return "Nothing";
    }

    public void setAllianceColor() {
        String alliance = getAllianceColor();
        if(alliance == "Blue") {
            solidBlue();
        }
        else if(alliance == "Red") {
            solidRed();
        }
        else {
            set(0.67); //color gold
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[LEDStrip] Alliance", getAllianceColor());
    }

}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Limelight extends SubsystemBase{


    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private double x = tx.getDouble(0.0);
    private double y = ty.getDouble(0.0);

    public Limelight() {
        
    }

    public Pose2d getPos() {
        Translation2d tPos = new Translation2d(x, y);
        Rotation2d rPos = new Rotation2d(x, y);
        Pose2d pPos = new Pose2d(tPos, rPos);
        
        return pPos;
    }
}

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  private final NetworkTable limelightTable;
  private double tv, tx, ta;
  private ArrayList<Double> targetList;
  private final int MAX_ENTRIES = 50;
  private final NetworkTableEntry isTargetValid, ledEntry;


  /**
   * Creates a new Vision.
   */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    targetList = new ArrayList<Double>(MAX_ENTRIES);
    isTargetValid = ShuffleboardInfo.getInstance().getTargetEntry();
    ledEntry = limelightTable.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);    
  }

  
}
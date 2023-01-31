package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  private final NetworkTable limelightTable;
  private double tv, tx, ty, ta;
  private final NetworkTableEntry ledEntry;
  private boolean targetExists = false;
  private final double kP = 0.03; //check
  private final double AIM_THRESHOLD = 1.0;


  /**
   * Creates a new Vision.
   */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight"); 
    ledEntry = limelightTable.getEntry("ledMode");
    
  }

  public double findDistance() {
    double targetOffsetAngle_Vertical = ty;
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; //TODO CHANGE
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; //TODO CHANGE
    // distance from the target to the floor
    double goalHeightInches = 60.0; //TODO CHANGE
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches-limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  public double autoAim()
  {
      double heading_error = tx; //Might need to be negative
      double steering_adjusment = 0.0;
      if(tx > AIM_THRESHOLD)
      {
          steering_adjusment = kP * heading_error;
      }
      else if(tx < AIM_THRESHOLD)
      {
          steering_adjusment = kP * heading_error;
      }
      double rotation = steering_adjusment;
      return rotation;
  }

  @Override
  public void periodic() {
    tv = limelightTable.getEntry("tv").getDouble(0);
    tx = limelightTable.getEntry("tx").getDouble(0);
    ty = limelightTable.getEntry("ty").getDouble(0);
    ta = limelightTable.getEntry("ta").getDouble(0);

    if (tv < 1) {
      targetExists = false;
    }
    else {
      targetExists = true;
    }

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);    
  }

  
}
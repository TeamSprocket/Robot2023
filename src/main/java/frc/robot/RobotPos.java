package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotPos {
    public static double posXMeters, posYMeters;

    public static Translation2d getPos() {
        return new Translation2d(posXMeters, posYMeters);
    }

    public static void Translation2d(Translation2d pose2d) {
        posXMeters = pose2d.getX();
        posYMeters = pose2d.getY();
    }

}
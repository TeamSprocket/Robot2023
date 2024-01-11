package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private double x = tx.getDouble(0.0);
    private double y = ty.getDouble(0.0);

    SwerveDrive swerveDrive;
    PIDController pidController;

    public Limelight(SwerveDrive swerveDrive) {

        this.swerveDrive = swerveDrive;
        this.pidController = new PIDController(Constants.Drivetrain.kLimelightAlignP,
                Constants.Drivetrain.kLimelightAlignI, Constants.Drivetrain.kLimelightAlignD);
        this.pidController.setSetpoint(0);
    }

    public void setSpeeds() {

        double tXOffset = table.getEntry("tx").getDouble(0.0);
        double output = -1 * pidController.calculate(tXOffset);

        ChassisSpeeds chassisSpeeds;

        if (Constants.Drivetrain.kIsFieldOriented) {

            double headingRad = Math.toRadians(swerveDrive.getHeading());
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, output, 0, new Rotation2d(headingRad));
        } else {

            chassisSpeeds = new ChassisSpeeds(0, output, 0);
        }

        SwerveModuleState[] moduleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveDrive.setModuleStates(moduleStates);
    }

    public void setAprilTagPipeline() {
        table.getEntry("pipeline").setNumber(0);
    }

    public double getXoffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public void changeAlliance() {
        DriverStation.Alliance color = DriverStation.getAlliance();

        if (color == DriverStation.Alliance.Blue) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
        } else if (color == DriverStation.Alliance.Red) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired");
        }
    }

    public void stop() {
        swerveDrive.stopModules();
    }

    public Pose2d getPos() {

        Translation2d tPos = new Translation2d(x, y);
        Rotation2d rPos = new Rotation2d(x, y);
        Pose2d pPos = new Pose2d(tPos, rPos);

        return pPos;
    }
}

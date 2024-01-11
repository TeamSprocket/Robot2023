package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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
        } 
        else {
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

    public static void getPose() {
        DoubleArraySubscriber x = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        double[] pose = x.get();
        SmartDashboard.putNumber("Limelight", pose[0]);
    }

    public void stop() {
        swerveDrive.stopModules();
    }
}

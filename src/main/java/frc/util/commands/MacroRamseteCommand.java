/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * A MacroCommand that uses a RamseteController to follow a Trajectory with a
 * differential drive. Intended to be run for a finite duration during the
 * autonomous period.
 * 
 * The command handles trajectory-following, PID calculations, and feedforwards
 * internally. This is intended to be a more-or-less "complete solution" that
 * can be used by teams without a great deal of controls expertise.
 * 
 * Advanced teams seeking more flexibility (for example, those who wish to use the
 * onboard PID functionality of a "smart" motor controller) may use the secondary
 * constructor that omits the PID and feedforward functionality, returning only
 * the raw wheel speeds from the RAMSETE controller.
 * 
 * In general, MacroRamseteCommands should aways be chained together to run in
 * sequence or parallel within AutonRoutines. They should never be run during
 * the teleoperated period.
 * 
 * MacroRamseteCommands are required to override the end() method, and cannot
 * override the initialize(), execute(), or isFinished() methods. The
 * initialize(), execute(), and isFinished() methods operate the superclass'
 * ramsete controller.
 */
public abstract class MacroRamseteCommand extends RamseteCommand {
  /**
   * Constructs a new MacroRamseteCommand that, when executed, will follow the
   * provided trajectory. PID control and feedforward are handled internally,
   * and outputs are scaled -12 to 12 representing units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon
   * completion of the path - this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose - use one of
   *                        the odometry classes to provide this.
   * @param controller      The RAMSETE controller used to follow the
   *                        trajectory.
   * @param feedforward     The feedforward to use for the drive.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param wheelSpeeds     A function that supplies the speeds of the left and
   *                        right sides of the robot drive.
   * @param leftController  The PIDController for the left side of the robot
   *                        drive.
   * @param rightController The PIDController for the right side of the robot
   *                        drive.
   * @param outputVolts     A function that consumes the computed left and right
   *                        outputs (in volts) for the robot drive.
   */
  public MacroRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose,
      RamseteController controller, SimpleMotorFeedforward feedForward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController, PIDController rightController,
      BiConsumer<Double, Double> outputVolts) {
    super(trajectory, pose, controller, feedForward, kinematics, wheelSpeeds,
        leftController, rightController, outputVolts);
  }

  /**
   * Constructs a new MacroRamseteCommand that, when executed, will follow the
   * provided trajectory. Performs no PID control and calculates no
   * feedforwards; outputs are the raw wheel speeds from the RAMSETE controller,
   * and will need to be converted into a usable form by the user.
   *
   * @param trajectory            The trajectory to follow.
   * @param pose                  A function that supplies the robot pose - use
   *                              one of the odometry classes to provide this.
   * @param follower              The RAMSETE follower used to follow the
*                                 trajectory.
   * @param kinematics            The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left
   *                              and right wheel speeds.
   */
  public MacroRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose,
      RamseteController follower, DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond) {
    super(trajectory, pose, follower, kinematics, outputMetersPerSecond);
  }

  // Called when the command is initially scheduled.
  // Subclasses cannot override this method.
  @Override
  public final void initialize() {
    super.isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Subclasses cannot override this method.
  @Override
  public final void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  // Subclasses are required to override this method.
  @Override
  public abstract void end(boolean interrupted);

  // Returns true when the command should end.
  // Subclasses cannot override this method.
  @Override
  public final boolean isFinished() {
    return super.isFinished();
  }
}

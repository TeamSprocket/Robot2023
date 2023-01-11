/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A Command intended to run instantaneously during either the autonomous or
 * teleoperated period. Will interrupt all commands requiring the same
 * Subsystem.
 * 
 * In general, InstantCommands should always be manually scheduled. They are
 * usually bound to controls such as joysticks or buttons, but they may also
 * be chained together to run in sequence or parallel within AutonRoutines.
 * 
 * InstantCommands are required to override the initialize() method, and cannot
 * override the execute(), end(), or isFinished() methods.
 */
public abstract class InstantCommand extends CommandBase {
  /**
   * Creates a new InstantCommand
   */
  public InstantCommand() {
  }

  // Called when the command is initially scheduled
  // Subclasses are required to override this method.
  @Override
  public abstract void initialize();

  // Called every time the scheduler runs while the command is scheduled.
  // Subclasses cannot override this method.
  @Override
  public final void execute() {
  }

  // Called once the command ends or is interrupted.
  // Subclasses cannot override this method.
  @Override
  public final void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  // Subclasses cannot override this method.
  @Override
  public final boolean isFinished() {
    return true;
  }
}

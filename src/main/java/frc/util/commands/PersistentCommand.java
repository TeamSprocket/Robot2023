/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A Command intended to be continuously run during the teleoperated period.
 * May be interrupted by a MacroCommand or MacroPIDCommand requiring the same
 * Subsystem.
 * 
 * In general, PersistentCommands should never be manually scheduled. Instead,
 * they should be set as the default Command of their respective Subsystem to
 * be automatically scheduled. They should also never be scheduled during the
 * autonomous period.
 * 
 * PersistentCommands are required to override the execute() and end() methods,
 * and cannot override the initialize() or isFinished() methods.
 */
public abstract class PersistentCommand extends CommandBase {
  /**
   * Creates a new PersistentCommand.
   */
  public PersistentCommand() {
  }

  // Called when the command is initially scheduled.
  // Subclasses cannot override this method.
  @Override
  public final void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Subclasses are required to override this method.
  @Override
  public abstract void execute();

  // Called once the command ends or is interrupted.
  // Subclasses are required to override this method.
  @Override
  public abstract void end(boolean interrupted);

  // This command should never finish intentionally.
  // Subclasses cannot override this method.
  @Override
  public final boolean isFinished() {
    return false;
  }
}

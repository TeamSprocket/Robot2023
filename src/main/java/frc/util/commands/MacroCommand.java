/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A Command intended to only run for a finite duration during either the
 * autonomous or teleoperated period. Will temporarily interrupt a
 * PersistentCommand or PersistentPIDCommand requiring the same
 * Subsystem.
 * 
 * In general, MacroCommands should always be manually scheduled. They are
 * usually bound to controls such as joysticks or buttons, but they may also
 * be chained together to run in sequence or parallel within AutonRoutines.
 * 
 * MacroCommands are required to override the initialize(), execute(), end(),
 * and isFinished() methods.
 */
public abstract class MacroCommand extends CommandBase {
  /**
   * Creates a new MacroCommand.
   */
  public MacroCommand() {
  }

  // Called when the command is initially scheduled
  // Subclasses are required to override this method.
  @Override
  public abstract void initialize();

  // Called every time the scheduler runs while the command is scheduled.
  // Subclasses are required to override this method.
  @Override
  public abstract void execute();

  // Called once the command ends or is interrupted.
  // Subclasses are required to override this method.
  @Override
  public abstract void end(boolean interrupted);

  // Returns true when the command should end.
  // Subclasses are required to override this method.
  @Override
  public abstract boolean isFinished();
}

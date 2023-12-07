// Coblic WristMovement(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Callepyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class WristMovement extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  private final Wrist wrist;
  private final double position;
  
  public ChangePosition (Wrist wrist, double position) {
    this.wrist = wrist;
    this.position = position;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.wristForward(100)
  }
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
}

package frc.robot.commands.instant;

import frc.robot.subsystems.PCH;
import frc.util.commands.InstantCommand;

public class CompressorOn extends InstantCommand {
  private final PCH PCH;
  /**
   * Creates a new CompressorOn.
   */
  public CompressorOn(PCH PCH) {
    this.PCH = PCH;

    addRequirements(PCH);
  }

  @Override
  public void initialize() {
    PCH.isEnabled();
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.superstructure.auton.RollClawTimed;
import frc.robot.commands.superstructure.auton.SetHighTimedCube;
import frc.robot.commands.superstructure.auton.SetHomeTimed;
import frc.robot.commands.superstructure.auton.SetLowCubeTimed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Wrist;

public class ParseAuton extends CommandBase {
  /** Creates a new ParseAuton. */
  double[][] auton = Constants.Auton.ROUTINE;


  int cur;
  Timer timer;
  double last = 0.0;
  SwerveDrive swerveDrive;
  Elevator elevator;
  Arm arm;
  Wrist wrist;
  Claw claw;

  public ParseAuton(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Wrist wrist, Claw claw) {
    this.swerveDrive = swerveDrive;
    cur = 0;
    timer = new Timer();

    this.elevator = elevator;
    this.arm = arm;
    this.wrist = wrist;
    this.claw = claw;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();

    double time = Math.round(timer.get() * 10) / 10.0;
        // System.out.println(time);
    if (time - (int) (time) != last) {
        last = time - (int) (time);

        SwerveModuleState[] states = { 
          new SwerveModuleState(auton[cur][0], new Rotation2d(auton[cur][4])), 
          new SwerveModuleState(auton[cur][1], new Rotation2d(auton[cur][5])), 
          new SwerveModuleState(auton[cur][2], new Rotation2d(auton[cur][6])), 
          new SwerveModuleState(auton[cur][3], new Rotation2d(auton[cur][7]))
        };

        swerveDrive.setModuleStates(states);


        cur++;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cur >= auton.length - 2;
  }
}

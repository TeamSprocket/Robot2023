package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Wrist.WristState;
import frc.util.GamePieces;


public class Superstructure extends SubsystemBase {
  
  SuperStructureState state = SuperStructureState.OFF;
  public enum SuperStructureState {
    HIGH_CONE,
    HIGH_CUBE,
    MID_CONE,
    MID_CUBE,

    INTAKE_CONE,
    INTAKE_FLOOR_CONE,
    INTAKE_CUBE,

    HOME,
    HOMING_CLEAR,
    MANUAL,
    OFF
  }

  Elevator elevator = new Elevator();
  Arm arm = new Arm();
  Wrist wrist = new Wrist();
  Intake intake = new Intake();


  public Superstructure() {}

  @Override
  public void periodic() {
    // ensureIdleIntake();
    runStateCases();

    putDebugInfo();
  }

  public void setState(SuperStructureState state) {
    this.state = state;
  }

  public SuperStructureState getState() {
      return state;
  }







  public void runStateCases() { 
    // SmartDashboard.putBoolean("Deporting", elevator.getElevatorHeight() < Constants.SuperstructureSetpoints.kElevatorCLEAR);

    switch (state) {
      case HOMING_CLEAR:
        // setElementStates(ElevatorState.HOMING_CLEAR, ArmState.HOMING_CLEAR, WristState.HOMING_CLEAR);

      case HOME:
        setElementStatesEnsureDeport(ElevatorState.HOME, ArmState.HOME, WristState.HOME);
        if (reachedState()) { //TODO: Currently overriding return to home with deport 
          state = SuperStructureState.OFF;
        }
        break;

      case MANUAL:
        setElementStates(ElevatorState.MANUAL, ArmState.MANUAL, WristState.MANUAL);
        break;
        // TODO---------------------------------------------------------------
      case OFF:
        setElementStates(ElevatorState.OFF, ArmState.OFF, WristState.OFF);
        break;
      
      //////////////////////////////////////////////////
      
      case HIGH_CONE:
        setElementStatesEnsureDeport(ElevatorState.HIGH_CONE, ArmState.HIGH_CONE, WristState.HIGH_CONE);
        break;

      case HIGH_CUBE:
        setElementStatesEnsureDeport(ElevatorState.HIGH_CUBE, ArmState.HIGH_CUBE, WristState.HIGH_CUBE);
        break;

      case MID_CONE:
        setElementStatesEnsureDeport(ElevatorState.MID_CONE, ArmState.MID_CONE, WristState.MID_CONE);
        break;

      case MID_CUBE:
        setElementStatesEnsureDeport(ElevatorState.MID_CUBE, ArmState.MID_CUBE, WristState.MID_CUBE);
        break;

      //////////////////////////////////////////////////
      
      case INTAKE_CONE:
        setElementStatesEnsureDeport(ElevatorState.INTAKE_CONE, ArmState.INTAKE_CONE, WristState.INTAKE_CONE);
        rollActiveIntake(GamePieces.CONE);
        break;

      case INTAKE_FLOOR_CONE:
        setElementStates(ElevatorState.INTAKE_FLOOR_CONE, ArmState.INTAKE_FLOOR_CONE, WristState.INTAKE_FLOOR_CONE);
        rollActiveIntake(GamePieces.CONE);
        break;

      case INTAKE_CUBE:
        setElementStatesEnsureDeport(ElevatorState.INTAKE_CUBE, ArmState.INTAKE_CUBE, WristState.INTAKE_CUBE);
        rollActiveIntake(GamePieces.CUBE);
        break;

    }
  }


  public boolean ensureDeported() {
    if (elevator.getElevatorHeight() < Constants.SuperstructureSetpoints.kElevatorCLEAR) {
      elevator.setState(ElevatorState.CLEAR);
      // System.out.println("Deporting...");
      return false;
    }
    return true;
  }

  public void setElementStatesEnsureDeport(ElevatorState elevatorState, ArmState armState, WristState wristState) {
    if (ensureDeported()) {
      elevator.setState(elevatorState);
      arm.setState(armState);
      wrist.setState(wristState);
    }
  }

  public void setElementStates(ElevatorState elevatorState, ArmState armState, WristState wristState) {
      elevator.setState(elevatorState);
      arm.setState(armState);
      wrist.setState(wristState);
  }

  public boolean reachedState() {
    SmartDashboard.putBoolean("Reached State", elevator.reachedStatePos());
    // return (elevator.reachedStatePos() && arm.reachedStatePos() && wrist.reachedStatePos());
    return false;
  }

  public void clearStickyFaults() {
    elevator.clearStickyFaults();
    arm.clearStickyFaults();
    wrist.clearStickyFaults();
    intake.clearStickyFaults();
  }

  // public void ensureIdleIntake() {
  //   if (state != SuperStructureState.OFF &&
  //          state != SuperStructureState.INTAKE_CONE &&
  //           state != SuperStructureState.INTAKE_FLOOR_CONE &&
  //            state != SuperStructureState.INTAKE_CUBE) {
  //     intake.setSpeed(Constants.Intake.kIdleIntakeSpeed);
  //   }
  // }

  public void rollActiveIntake(GamePieces gamePiece) {
    double speed = Constants.Intake.kActiveIntakeSpeed;
    if (gamePiece == GamePieces.CUBE) {
      speed *= -1;
    }

    intake.setSpeed(speed);
  }

  public Intake tempFunctGetIntake() {  
    return intake;
  }





  

  

  public void putDebugInfo() {
    SmartDashboard.putString("Superstructure State", state.toString());
  }

}

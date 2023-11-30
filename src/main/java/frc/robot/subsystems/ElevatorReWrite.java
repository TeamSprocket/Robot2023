package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ElevatorReWrite extends SubsystemBase {
    private final CANSparkMax elevatorLeft = new CANSparkMax(RobotMap.Elevator.ELEVATOR_LEFT, MotorType.kBrushless); // port
                                                                                                                     // and
                                                                                                                     // type
    private final CANSparkMax elevatorRight = new CANSparkMax(RobotMap.Elevator.ELEVATOR_RIGHT, MotorType.kBrushless);
    private final PIDController elevatorPID = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI,
            Constants.Elevator.kD);
    private final RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private final RelativeEncoder elevatorRightEnder = elevatorRight.getEncoder();
    // public double currentSpeed;
    // private double targetSpeed;
    // public double PIDResultSpeed;

    public ElevatorReWrite() {
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();
        elevatorLeftEncoder.setPosition(0);
        elevatorRightEnder.setPosition(0);

        // set motor follow
        elevatorLeft.setInverted(true);
        elevatorRight.follow(elevatorLeft);

        // eletrical
        elevatorLeft.enableVoltageCompensation(8);
        elevatorRight.enableVoltageCompensation(8);
    }


    public void clearStickyFaults() {
        elevatorLeft.clearFaults();
        elevatorRight.clearFaults();
    }

    public void setHeight(double target) {
        elevatorPID.setSetpoint(target);
    }

    private double getHeight() {
        
        return elevatorLeftEncoder.getPosition() * Math.PI * 2 /Constants.Elevator.kElevatorGearRatio;
    }

    @Override
    public void periodic() {
        double calc = elevatorPID.calculate(getHeight());
        elevatorLeft.set(calc);
        System.out.println(calc);

    }



//     public void setTarget(double target) {
//         targetSpeed = target;

//     }

//     public void periodicSetMotor() {
//         if ((Double) targetSpeed!=null) {
//             // calculate point, target_point
//         double PIDResultSpeed = elevatorPID.calculate(currentSpeed, targetSpeed);
//         elevatorLeft.set(PIDResultSpeed);}

//     }
// +

//     public double calculateHeight(double gearRadius) {
//         // 2r pi * rotation
//         return elevatorLeftEncoder.getPosition() * Math.PI * 2 * gearRadius;
//     }

//     public void fastPeriodic() {
//         clearStickyFaults();
//         // may be wrong method
//         currentSpeed = elevatorLeft.get();
//         periodicSetMotor();
//     }

//     public void slowPeriodic() {
//         System.out.println("Current speed: " + currentSpeed);
//         System.out.println("Target speed: " + targetSpeed);
//         System.out.println("PID decision speed: " + PIDResultSpeed);
//         System.out.println("Rotation difference between encoder: "
//                 + Math.abs(elevatorLeftEncoder.getPosition() - elevatorRightEnder.getPosition()));
//     }

}


// package frc.robot.commands.auton;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.commands.SwerveDriveCmd;
// import frc.robot.subsystems.SwerveDrive;

// public class BalanceOnChargeStationHorizontal extends CommandBase {

//   private final PIDController controller; 
//   private final SwerveDrive swerveDrive;
//   private final Timer timer;
//   private final double duration;
  
//   private final ADIS16470_IMU gyro = new ADIS16470_IMU();

//   /** Creates a new BalanceOnChargeStation. */
//   public BalanceOnChargeStationHorizontal(SwerveDrive swerveDrive, double duration) {
//     controller = new PIDController(Constants.Auton.kPBalance, Constants.Auton.kIBalance, Constants.Auton.kDBalance);
//     this.swerveDrive = swerveDrive;
//     timer = new Timer();
//     this.duration = duration;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double roll = gyro.getYComplementaryAngle();
//     SmartDashboard.putNumber("ROLL", roll);

//     double output = controller.calculate(roll, 0);

//     new SwerveDriveCmd(
//       swerveDrive,
//       () -> 0.0,
//       () -> output,
//       () -> 0.0
//     );

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if (timer.get() >= duration) {
//       return true;
//     }
//     return false;
//   }
// }

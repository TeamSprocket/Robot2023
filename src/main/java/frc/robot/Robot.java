// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.dacubeking.AutoBuilder.robot.NetworkAuto;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.auton.TemplateAuto;
import frc.auton.Test;
import frc.util.OrangeUtility;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable autoDataTable = instance.getTable("autodata");
    NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");

    NetworkTableEntry enabled = autoDataTable.getEntry("enabled");
    NetworkTableEntry pathProcessingStatusEntry = autoDataTable.getEntry("processing");
    NetworkTableEntry pathProcessingStatusIdEntry = autoDataTable.getEntry("processingid");

    private final Lock networkAutoLock = new ReentrantLock();
    NetworkAuto networkAuto;
    

    String lastAutoPath = null;

    ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    //Auto
    // WeakSide weakSideAuto = new WeakSide();
    Test testAuto = new Test();
    // TwoBall twoBallAuto;
    // FiveBall fiveBallAuto; 
    TemplateAuto selectedAuto;
    Thread autoThread;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    

    private final RobotContainer robotContainer = new RobotContainer();
    // private final SendableChooser<String> autoChooser = new SendableChooser<>();
    // private final SendableChooser<String> sideChooser = new SendableChooser<>();

    public Robot() {
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        try{
            testAuto = new Test();
        }
        finally{}

        selectedAuto = testAuto;    
        // if (autoPath.getString(null) != null) {
        //     autoPathListener.accept(new EntryNotification(NetworkTableInstance.getDefault(), 1, 1, "", null, 12));
       
        // }

        // autoPath.addListener(autoPathListener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);



        // startSubsystems();
        // drive.resetHeading();
        OrangeUtility.sleep(50);
        // odometry.setOdometry(new Pose2d());
        // VisionManager.getInstance().toggleLimelight(false);
        autoChooser.setDefaultOption("default","two"); //Test if this works lololol
        autoChooser.addOption("five ball","five");
        // autoChooser.addOption("weak side","weak");
        autoChooser.addOption("test40", "testAuto");
        SmartDashboard.putData(autoChooser);
        AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

    //     //Ensure the second String is the name of the folder where your sided autos are located
    //     sideChooser.setDefaultOption("Blue", "blue"); 
    //     sideChooser.addOption("Red", "red");

    //     SmartDashboard.putData("Auto choices", autoChooser);
    //     SmartDashboard.putData("Red or Blue", sideChooser);  

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        enabled.setBoolean(true);

        networkAutoLock.lock();
        try {
             if(autoChooser.getSelected() =="testAuto")
                 selectedAuto = testAuto;
             
             else if(autoChooser.getSelected() == "default")
                 selectedAuto = testAuto;
 
                 SmartDashboard.putString("which auto?", autoChooser.getSelected());
             
         } finally {
             networkAutoLock.unlock();
         }
         
         if( selectedAuto != null){
             selectedAuto.reset();
 
             autoThread = new Thread(selectedAuto);
             autoThread.start();
         }
 

    //     String autoName = autoChooser.getSelected();
    //     if (autoName == null) {
    //         autoName = "1ball"; // Default auto if none is selected
    //     }
    //     // If it can't find a sided auto it will try to find a non-sided auto
    //     AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.getSelected(), true); // The last boolean is about allowing network autos to run, keep this set to true unless you have a reason to disable them.
    // }
        // Command auton = robotContainer.getAutonomousCommand();

        // if(auton!=null) {
        //     auton.schedule();
        // }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once each time the robot enters teleoperated mode. */ 
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    
    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;





/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  //I formatted like this because it looks better in my opinion (not like way super long) - Orange
  String[] autonomousList = {
  "STAGE 3-Note Auto -- MIDDLE START (GRABS STAGE-SIDE NOTE)", // 0
  "AMP 3-Note Auto -- MIDDLE START (GRABS AMP-SIDE NOTE)", // 1
  "Inner Side Starter -- CLOSE TO STAGE START", // 2
  "Outer Side Starter -- CLOSE TO AMP START", // 3
  "Chaos -- CLOSE TO STAGE START", // 4
  "Driving PID Test Path -- TESTING AUTO", // 5
  "Turning PID Test Path -- TESTING AUTO" // 6
  };


  String autoSelected;
  //private final Compressor c_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  //private CANSparkMax m_Claw_r = new CANSparkMax(16, MotorType.kBrushless);
  private CANSparkMax m_Claw_l = new CANSparkMax(25, MotorType.kBrushless);
  private final AnalogInput ultrasonic = new AnalogInput(0); 
  //private AnalogInput sonar = new AnalogInput(0);

  private final PWMSparkMax p_Lights = new PWMSparkMax(0);

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  Joystick stick;
  Joystick stick2;

  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  final double DRIVE_P = 1;
  final double DRIVE_D = 0.0;
  PIDController driveController = new PIDController(DRIVE_P, 0, DRIVE_D);

  PhotonCamera camera = new PhotonCamera("cam");

  double range = 0;

  final double CLOSE_X = 38;
  final double FAR_X = 52.5;

  boolean primed = false;
  boolean isUp;

  boolean spinUp = false;

  // double ff = 0.4;
  // double p = 0.25;
  // double i = 0.000002;
  // double d = 0.25;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // SmartDashboard.putNumber("DB/Slider 0", p);
    // SmartDashboard.putNumber("DB/Slider 1", i);
    // SmartDashboard.putNumber("DB/Slider 2", d);
    // SmartDashboard.putNumber("DB/Slider 3", ff);

    SmartDashboard.putStringArray("Auto List", autonomousList);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
  // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
        stick = m_robotContainer.m_driverController;
        stick2 = m_robotContainer.m_driverController2;
    //isUp = !m_robotContainer.m_robotShooter.isShooterUp();
    //m_Claw_l.set(-1);
    //m_Claw_r.set(-1);

    
    
    // switch(DriverStation.getAlliance().get()){
    //   case Red:
    //   p_Lights.set(-.35);
    //   break;
    //   case Blue:
    //   p_Lights.set(0.89);
    //   break;
    //   default:
    //   p_Lights.set(-.99);
    //   break;
    // }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    autoSelected = SmartDashboard.getString("Auto Selector", "None");
    
    
    switch(autoSelected) {
      case "STAGE 3-Note Auto -- MIDDLE START (GRABS STAGE-SIDE NOTE)": 
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(0);
      break;
      case "AMP 3-Note Auto -- MIDDLE START (GRABS AMP-SIDE NOTE)": 
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(1);
      break;
      case "Inner Side Starter -- CLOSE TO STAGE START":
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(2);
      break;
      case "Outer Side Starter -- CLOSE TO AMP START":
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(3);
      break;
      case "Chaos -- CLOSE TO STAGE START":
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(4);
      break;
      case "Driving PID Test Path -- TESTING AUTO":
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(5);
      break;
      case "Turning PID Test Path -- TESTING AUTO":
      m_autonomousCommand = m_robotContainer.autonomousTestingCommands(6);
      break;
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //hey shiva this is a wip thing that me n jared dont know what to do with .
    /*
    double w_FL = m_robotContainer.m_robotDrive.m_frontLeft.getState().speedMetersPerSecond;
    double w_FR = m_robotContainer.m_robotDrive.m_frontRight.getState().speedMetersPerSecond;
    double w_BL = m_robotContainer.m_robotDrive.m_rearLeft.getState().speedMetersPerSecond;
    double w_BR = m_robotContainer.m_robotDrive.m_rearRight.getState().speedMetersPerSecond;

    if(Math.signum(w_FL)>=0){
    m_robotContainer.m_robotDrive.m_frontLeft.m_drivingPIDController.setFF(Math.signum(w_FL)*Constants.ModuleConstants.kDrivingFF);
    m_robotContainer.m_robotDrive.m_frontRight.m_drivingPIDController.setFF(Math.signum(w_FR)*Constants.ModuleConstants.kDrivingFF);
    m_robotContainer.m_robotDrive.m_rearLeft.m_drivingPIDController.setFF(Math.signum(w_BL)*Constants.ModuleConstants.kDrivingFF);
    m_robotContainer.m_robotDrive.m_rearRight.m_drivingPIDController.setFF(Math.signum(w_BR)*Constants.ModuleConstants.kDrivingFF);
    }
    else{
    m_robotContainer.m_robotDrive.m_frontLeft.m_drivingPIDController.setFF(0);
    m_robotContainer.m_robotDrive.m_frontRight.m_drivingPIDController.setFF(0);
    m_robotContainer.m_robotDrive.m_rearLeft.m_drivingPIDController.setFF(0);
    m_robotContainer.m_robotDrive.m_rearRight.m_drivingPIDController.setFF(0);
    }
    */
  }

  @Override
  public void teleopInit() {
    Constants.DriveConstants.kMaxSpeedMetersPerSecond = 4.5;
    Constants.DriveConstants.kMaxAngularSpeed = 3.5 * Math.PI;
    p_Lights.set(0.83);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

   // m_robotContainer.m_robotShooter.compressorSwitch();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double rawValue = ultrasonic.getValue();  

    if(stick.getRawButton(3)) {
      m_robotContainer.m_robotShooter.revUp(0.4, 2.75);
      spinUp = true;
    }
    else {
      spinUp = false;
    }
    // Constants.ModuleConstants.kDrivingP = SmartDashboard.getNumber("DB/Slider 0", p);
    // Constants.ModuleConstants.kDrivingI = SmartDashboard.getNumber("DB/Slider 1", i);
    // Constants.ModuleConstants.kDrivingD = SmartDashboard.getNumber("DB/Slider 2", d);
    // Constants.ModuleConstants.kDrivingFF = SmartDashboard.getNumber("DB/Slider 3", ff);
    // System.out.println();

    if(stick.getRawAxis(3) > .3) {
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 7.5;
    } else if (stick.getRawAxis(2) > .3) {
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 2;
      Constants.DriveConstants.kMaxAngularSpeed = Math.PI;
    } else {
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 4.5;
      Constants.DriveConstants.kMaxAngularSpeed = 2 * Math.PI;
    }

    //if (m_robotContainer.m_robotShooter.limitSwitch.get()) {
    //  System.out.println("Being Pressed");
    //} else {
    //  System.out.println("Not Being Pressed");
    //}

    if(stick.getRawButton(1)) {
      //m_Claw_r.set(-.5);
      m_Claw_l.set(-.5);
    } else if(stick.getRawButton(2)) {
      //m_Claw_r.set(.5);
      m_Claw_l.set(.5);
      //System.out.println("down");
    } else {
      //m_Claw_r.set(0);
      m_Claw_l.set(0);
      //System.out.println("not moving");
    }
    

    
    var result = camera.getLatestResult();
    if(camera.isConnected()){
      System.out.println("Connecterd");
      if(result.hasTargets()){
        System.out.println("Has Targetes");
      } else {
        System.out.println("No Targetes");
      }
    } else {
      System.out.println("Not Connecterd");
    }


    // if (result.hasTargets()) {
    //   range = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(13), Units.inchesToMeters(56.25), Units.degreesToRadians(33), Units.degreesToRadians(result.getBestTarget().getPitch()));
    //   Constants.DriveConstants.rotationSpeed = turnController.calculate(result.getBestTarget().getYaw()-5, 2.4);
    //   if (Math.abs(range - Units.inchesToMeters(FAR_X)) > .05 && !isUp) {
    //     Constants.DriveConstants.driveSpeedX = driveController.calculate(range, Units.inchesToMeters(FAR_X));
    //     primed = false;
    //   }
    //   else if (Math.abs(range - Units.inchesToMeters(CLOSE_X)) > .15 && isUp) {
    //     Constants.DriveConstants.driveSpeedX = driveController.calculate(range, Units.inchesToMeters(CLOSE_X));
    //     primed = false;
    //   }
    //   else {
    //     Constants.DriveConstants.driveSpeedX = 0;
    //     if(stick.getRawButton(3)){
    //       primed = true;
    //     }
    //     else {
    //       primed = false;
    //     }
        
    //   }
    // }
    //   else {
    //   Constants.DriveConstants.rotationSpeed /= 5;
    //   Constants.DriveConstants.driveSpeedX /= 1.5;
    // }

   
    
    // for picking up note (lb = out, rb = in)
    if (stick2.getRawButton(6) == true && stick.getRawButton(5) == false && !spinUp) {
      m_robotContainer.m_robotShooter.intakeNote(0.2);
    } else if (stick2.getRawButton(5) == true && stick.getRawButton(6) == false) {
      m_robotContainer.m_robotShooter.runIntake(-0.15);
      m_robotContainer.m_robotShooter.runBelly(-1);
    } else if (!spinUp) {
      m_robotContainer.m_robotShooter.runIntake(0);
      m_robotContainer.m_robotShooter.runBelly(0);
    }

    

    // for shooting note (Y = speaker)
    if(stick2.getRawButton(4) == true && stick2.getRawButton(2) == false) {  //for Amp
      m_robotContainer.m_robotShooter.shoot(0.05, 1, m_robotContainer.m_robotDrive, 1.66);
    } 
    else if(stick2.getRawButton(2) == true && stick2.getRawButton(4) == false) {  //for Speaker
      if (isUp) {
        if(!spinUp) {
          m_robotContainer.m_robotShooter.shoot(0.4, 1, m_robotContainer.m_robotDrive, 2.75);
        }
        else {
          m_robotContainer.m_robotShooter.runBelly(1);
          m_robotContainer.m_robotShooter.runIntake(1);
        }
      }
      else {
        if(!spinUp){
          m_robotContainer.m_robotShooter.shoot(0.625, 1, m_robotContainer.m_robotDrive, 1.9);
        }
        else {
          m_robotContainer.m_robotShooter.runBelly(1);
          m_robotContainer.m_robotShooter.runIntake(1);
        }
      } 
    }  
    // for picking up through shooter (A button)
    if (stick2.getRawButton(1) == true && !spinUp) {
      m_robotContainer.m_robotShooter.runShooter(-0.05);
    } 
    else if (!spinUp) {
      m_robotContainer.m_robotShooter.runShooter(0);
    }

     /*arms up
    if(stick2.getRawAxis(3)>=.5) {
       m_robotContainer.m_robotShooter.shooterUp();
       isUp = true;
    } 

    //arms down 
     if(stick2.getRawAxis(2)>=.5) {
       m_robotContainer.m_robotShooter.shooterDown();
       isUp = false;

    }

  */

    if (primed) {
      p_Lights.set(0.22);
      System.out.println("Primed");
    }
    else if(rawValue<=250 && !primed){
      p_Lights.set(0.65);
    }
    else{
      p_Lights.set(0.87);
    }

     //System.out.println(sonar.getAverageVoltage());

  }


  @Override
  public void testInit() {
    p_Lights.set(0.88);
    System.out.print("get out of test mode");
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    double rawValue = ultrasonic.getValue();
    System.out.println(rawValue);

  }
}

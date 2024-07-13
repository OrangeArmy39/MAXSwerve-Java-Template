// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.*;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ShooterSubsystem m_robotShooter = new ShooterSubsystem();

  // The driver's controller
  public Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort0);
  public Joystick m_driverController2 = new Joystick(OIConstants.kDriverControllerPort1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() {

    NamedCommands.registerCommand("ShootSpeaker", m_robotShooter.shooterCommand(0.4, 1, m_robotDrive, 2.75));
    NamedCommands.registerCommand("RunIntake", m_robotShooter.intakeCommand(.130));
    NamedCommands.registerCommand("StopIntake", m_robotShooter.intakeCommand(0));
    NamedCommands.registerCommand("NotifyEnded", m_robotShooter.notifyEndedCommand());
    NamedCommands.registerCommand("StopShooter", m_robotShooter.stopAllCommand());
    //NamedCommands.registerCommand("LowerShooter", m_robotShooter.shooterDownCommand());
    //NamedCommands.registerCommand("RaiseShooter", m_robotShooter.shooterUpCommand());
    NamedCommands.registerCommand("StopShooting", m_robotShooter.stopShooterCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband),
                true/*, true*/),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
         .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
             m_robotDrive));
    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(
            () -> m_robotDrive.aimShooter(Constants.DriveConstants.rotationSpeed, Constants.DriveConstants.driveSpeedX),
             m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command autonomousTestingCommands(int autothingy) {
    var automice = 7; //MAKE SURE TO UPDATE THIS WHEN ADDING AUTOS!!!!!!!1
    String autoArray[] = new String[automice];
    autoArray[0] = "STAGE - 3-Note Auto"; 
    autoArray[1] = "AMP - 3-Note Auto";
    autoArray[2] = "Inner Side Starter";
    autoArray[3] = "Outer Side Starter";
    autoArray[4] = "Chaos";
    autoArray[5] = "Driving PID Test Path - Auto";
    autoArray[6] = "Turning PID Test Path - Auto";
    return new PathPlannerAuto(autoArray[autothingy]);
  }
}

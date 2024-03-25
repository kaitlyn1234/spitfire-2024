// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

 // private final DriveSubsystem m_robotDrive2 = new DriveSubsystem();


  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

// 3 NOTE AUTO
  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);
    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.5, 0.02), new Translation2d(1, -0.02)),
        new Pose2d(1.6, 0, new Rotation2d(0)),
        config);

   /*  Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.5, -0.10), new Translation2d(1, 0.10)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5052, 0, new Rotation2d(0)),
        config);*/

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }



// 2 NOTE AUTO?
  public Command getAutonomousCommand2() {

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

  Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.6, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, -0.02), new Translation2d(0.5, 0.02)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        backTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    m_robotDrive.resetOdometry(backTrajectory.getInitialPose());

    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  
  // 3 Note Auto
  public Command testAutoCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory secondNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.5, 0.02), new Translation2d(-1, -0.02)),
        new Pose2d(-1.6, 0, new Rotation2d(0)),
        config);

    Trajectory thirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.6, 0, new Rotation2d(0)),
        List.of(new Translation2d(-1, -0.02), new Translation2d(-0.5, 0.02)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        secondNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        thirdNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    return swerveControllerCommand.andThen(swerveControllerCommand2.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }

  public Command testAutoCommand2() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory secondNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.01, 0.01), new Translation2d(-0.015, 0.015)),
        new Pose2d(-0.02, 0.02, new Rotation2d(-0.75)),
        config);

    Trajectory thirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.02, 0.02, new Rotation2d(-0.75)),
        List.of(new Translation2d(-0.015, 0.015), new Translation2d(-0.01, 0.01)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        secondNoteTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,


        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        thirdNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    return swerveControllerCommand.andThen(swerveControllerCommand2.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }


// wHAT even is this
    public Command testAutoCommand3() {
       TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kAlternateMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory secondNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.5, 0.02), new Translation2d(-1, -0.02)),
        new Pose2d(-1.6, 0, new Rotation2d(0)),
        config);

    Trajectory thirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.6, 0, new Rotation2d(0)),
        List.of(new Translation2d(-1, -0.02), new Translation2d(-0.5, 0.02)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);
    
    Trajectory fourthNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.5, 0.5), new Translation2d(-0.75, 1.5)),
        new Pose2d(-1.3, 1.45, new Rotation2d(0)),
        config);

    // -1.3 & 1.6 *Didn't pick up

    Trajectory fifthNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.3, 1.45, new Rotation2d(0)),//1.7
        List.of(new Translation2d(-0.9, 1.2), new Translation2d(-0.5, 0.5)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);
    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        secondNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        thirdNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,
 
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    
    SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
        fourthNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,
        
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
        fifthNoteTrajectory,
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    return swerveControllerCommand.andThen(swerveControllerCommand2.andThen(swerveControllerCommand3.andThen(swerveControllerCommand4.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)))));
  }

// WHAT
  public Command testAutoCommand4() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory secondNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-0.5, 0.02), new Translation2d(-1, -0.02)),
        new Pose2d(-1.6, 0, new Rotation2d(0)),
        config);

    Trajectory thirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.6, 0, new Rotation2d(0)),
        List.of(new Translation2d(-1, -0.02), new Translation2d(-0.5, 0.02)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        secondNoteTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        thirdNoteTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    return swerveControllerCommand.andThen(swerveControllerCommand2.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.commands.AutoArmDownCommand;
import frc.commands.AutoArmUpCommand;
import frc.commands.AutoScoreCommand;
import frc.commands.AutoStopScoreCommand;
import frc.commands.ManipulateArmCommand;
import frc.commands.ManipulateIntakeCommand;
import frc.commands.ResetGyroCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  private SequentialCommandGroup m_scoreAndBalance = setAutonomousCommand(-3.0, AutoConstants.kMaxSpeedMetersPerSecond/2.0);
  private SequentialCommandGroup m_scoreAndDrive = setAutonomousCommand(-6.0, AutoConstants.kMaxSpeedMetersPerSecond);
  private SequentialCommandGroup m_scoreAndStay = setAutonomousCommand(0.0, AutoConstants.kMaxSpeedMetersPerSecond);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

//The AuxController
  Joystick m_auxController = new Joystick(OIConstants.kAuxControllerPort);

  Trigger collectConeButton = new Trigger(() -> m_auxController.getRawAxis(2) > 0.3);
  Trigger scoreConeButton = new JoystickButton(m_auxController, 5);

  Trigger armUpButton = new JoystickButton(m_auxController, 4);
  Trigger armDownButton = new JoystickButton(m_auxController, 1);
  Trigger armMidButton = new JoystickButton(m_auxController, 2);

  //Trigger zeroHeadingButton = new JoystickButton(m_driverController, 1);

  Trigger collectCubeButton = new Trigger(() -> m_auxController.getRawAxis(3) > 0.3);
  Trigger scoreCubeButton = new JoystickButton(m_auxController, 6);

  Trigger goSlowButton = new JoystickButton(m_driverController, 6);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(m_chooser);

    m_chooser.setDefaultOption("Score and stop.", m_scoreAndStay);
    m_chooser.setDefaultOption("Score and balance.", m_scoreAndBalance);
    m_chooser.setDefaultOption("Score and drive.", m_scoreAndDrive);


    // Configure default commands
     m_robotDrive.setDefaultCommand(
         // The left stick controls translation of the robot.
         // Turning is controlled by the X axis of the right stick.
         
         new RunCommand(
             () ->
                 m_robotDrive.drive(
                     m_driverController.getRawAxis(1),
                     m_driverController.getRawAxis(0),
                     m_driverController.getRawAxis(4),
                     true,
                     10.0),
             m_robotDrive));



      m_intake.setDefaultCommand(new ManipulateIntakeCommand(m_intake, IntakeMode.OFF));
      m_arm.setDefaultCommand(new ManipulateArmCommand(m_arm, ArmMode.OFF));

    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    collectConeButton.whileTrue(new ManipulateIntakeCommand(m_intake, IntakeMode.COLLECT_CONE));
    scoreConeButton.whileTrue(new ManipulateIntakeCommand(m_intake, IntakeMode.SCORE_CONE));
    collectCubeButton.whileTrue(new ManipulateIntakeCommand(m_intake, IntakeMode.COLLECT_CUBE));
    scoreCubeButton.whileTrue(new ManipulateIntakeCommand(m_intake, IntakeMode.SCORE_CUBE));
    armDownButton.whileTrue(new ManipulateArmCommand(m_arm, ArmMode.DOWN));
    armMidButton.whileTrue(new ManipulateArmCommand(m_arm, ArmMode.MID));
    armUpButton.whileTrue(new ManipulateArmCommand(m_arm, ArmMode.UP));
    //zeroHeadingButton.onTrue(new ResetGyroCommand(m_robotDrive));


    goSlowButton.whileTrue(         new RunCommand(
      () ->
          m_robotDrive.drive(
              m_driverController.getRawAxis(1),
              m_driverController.getRawAxis(0),
              m_driverController.getRawAxis(4),
              true,
              3.0),
      m_robotDrive));
    
  }

  public Command getAutonomousCommand()
  {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup setAutonomousCommand(double dist, double speed) {

    double distance = dist;
    
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                speed,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d().fromDegrees(180)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(distance/3.0, .01), new Translation2d(2.0*distance/3.0, -0.01)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(distance, 0, new Rotation2d().fromDegrees(180)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    return new SequentialCommandGroup(
    new AutoArmDownCommand(m_arm),
    new WaitCommand(1.0),
    //new AutoScoreCommand(m_intake),
    //new AutoStopScoreCommand(m_intake),
    new AutoArmUpCommand(m_arm).withTimeout(1.0),
    new WaitCommand(1.0),
    swerveControllerCommand, 
    //new RunCommand(() -> m_robotDrive.drive(0, 0, .2, false, 3.0)),
    //new WaitCommand(0.5),
    new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, 3.0)));

    
    
    //new AutoArmDownCommand(m_arm).
    //andThen(new WaitCommand(2.0)).
    //andThen(new AutoScoreCommand(m_intake)).
    //andThen(new WaitCommand(2.0)).
    //andThen(new ManipulateIntakeCommand(m_intake, IntakeMode.OFF)).withTimeout(2.0).
    //andThen(new WaitCommand(2.0)).
    //andThen(swerveControllerCommand).
    //andThen(() -> m_robotDrive.drive(0, 0, 0, false, 3.0));
  }
}

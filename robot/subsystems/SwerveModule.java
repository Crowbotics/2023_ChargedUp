// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANCoder m_canCoder;

  SwerveModuleState m_desiredState;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int canCoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_canCoder = new CANCoder(canCoderChannel);
        
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_turningMotor.setInverted(true);
    
    m_driveEncoder = m_driveMotor.getEncoder();

    //m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);
    m_turningEncoder = m_turningMotor.getEncoder();
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse/60);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse/60);

    

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_desiredState = new SwerveModuleState();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(getRads()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(getRads()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = desiredState;
      
      //  SwerveModuleState.optimize(m_desiredState, new Rotation2d(getRads()));


    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getRads(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  public double getDegrees()
  {
    return m_canCoder.getAbsolutePosition();
  }

  public double getRads()
  {
    return m_canCoder.getAbsolutePosition() / 360.0 * 2*Math.PI;
  }

      /**
     * Find the reverse of a given angle (i.e. pi/4->7pi/4)
     * @param radians the angle in radians to reverse
     * @return the reversed angle
     */
    private double findRevAngle(double radians) {
      return (Math.PI * 2 + radians) % (2 * Math.PI) - Math.PI;
  }

  /**
   * Finds the distance in ticks between two setpoints
   * @param setpoint initial/current point
   * @param position desired position
   * @return the distance between the two point
   */
  private double getDistance(double setpoint, double position) {
      return Math.abs(setpoint - position);
  }
  
    /**
     * Optimize the swerve module state by setting it to the closest equivalent vector
     * @param original the original swerve module state
     * @return the optimized swerve module state
     */
    private SwerveModuleState optimizeState(SwerveModuleState original) {
      // Compute all options for a setpoint
      double position = getRads();
      double setpoint = original.angle.getRadians();
      double forward = setpoint + (2 * Math.PI);
      double reverse = setpoint - (2 * Math.PI);
      double antisetpoint = findRevAngle(setpoint);
      double antiforward = antisetpoint + (2 * Math.PI);
      double antireverse = antisetpoint - (2 * Math.PI);

      // Find setpoint option with minimum distance
      double[] alternatives = { forward, reverse, antisetpoint, antiforward, antireverse };
      double min = setpoint;
      double minDistance = getDistance(setpoint, position);
      int minIndex = -1;
      for (int i = 0; i < alternatives.length; i++) {
          double dist = getDistance(alternatives[i], position);
          if (dist < minDistance) {
              min = alternatives[i];
              minDistance = dist;
              minIndex = i;
          }
      }

      // Figure out the speed. Anti- directions should be negative.
      double speed = original.speedMetersPerSecond;
      if (minIndex > 1) {
          speed *= -1;
      }

      return new SwerveModuleState(speed, new Rotation2d(min));
  }

}
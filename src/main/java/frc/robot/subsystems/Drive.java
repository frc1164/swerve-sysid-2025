// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SparkMax m_LeftFrontMotor = new SparkMax(DriveConstants.kLeftFrontMotor1Port, MotorType.kBrushless);
  private final SparkMax m_LeftBackMotor = new SparkMax(DriveConstants.kLeftBackMotor1Port, MotorType.kBrushless);
  // The motors on the right side of the drive.
  private final SparkMax m_RightFrontMotor = new SparkMax(DriveConstants.kRightFrontMotor1Port, MotorType.kBrushless);
  private final SparkMax m_RightBackMotor = new SparkMax(DriveConstants.kRightBackMotor1Port, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_LeftFrontMotor, m_RightFrontMotor);

  // The left-side drive encoder
  private final RelativeEncoder m_LeftFrontEncoder = m_LeftFrontMotor.getEncoder();

  // The right-side drive encoder
  private final RelativeEncoder m_RightFrontEncoder = m_RightFrontMotor.getEncoder();

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final SparkMaxConfig frontLeftSparkMaxConfig = new SparkMaxConfig();
  private final SparkMaxConfig frontRightSparkMaxConfig = new SparkMaxConfig();
  private final SparkMaxConfig backLeftSparkMaxConfig = new SparkMaxConfig();
  private final SparkMaxConfig backRightSparkMaxConfig = new SparkMaxConfig();

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Voltage volts) -> {
            m_LeftFrontMotor.setVoltage(volts.in(Volts));
            m_RightFrontMotor.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_LeftFrontMotor.getAppliedOutput() * m_LeftFrontMotor.getBusVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(m_LeftFrontEncoder.getPosition()
                    * DriveConstants.kWheelDiameterMeters * Math.PI * DriveConstants.kDriveMotorGearRatio, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_LeftFrontEncoder.getVelocity() * DriveConstants.kWheelDiameterMeters
                        * Math.PI * DriveConstants.kDriveMotorGearRatio * 1 / 60, MetersPerSecond));

            // Record a frame for the right motors. Since these share an encoder, we
            // consider
            // the entire group to be one motor.
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_RightFrontMotor.getAppliedOutput() * m_RightFrontMotor.getBusVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(m_RightFrontEncoder.getPosition()
                    * DriveConstants.kWheelDiameterMeters * Math.PI * DriveConstants.kDriveMotorGearRatio, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(m_RightFrontEncoder.getVelocity() * DriveConstants.kWheelDiameterMeters
                        * Math.PI * DriveConstants.kDriveMotorGearRatio * 1 / 60, MetersPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("drive")
          this));
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public static final double kDriveMotorGearRatio = 1 / 6.75;
  public static final double kTurningMotorGearRatio = 7.00 / 150.00;// 150.00 / 7.00;
  public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
  public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
  public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
  public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

  /** Creates a new Drive subsystem. */
  public Drive() {
    // m_LeftFrontMotor.setInverted(false);
    // m_LeftBackMotor.setInverted(false);
    // m_RightFrontMotor.setInverted(false);
    // m_RightBackMotor.setInverted(false);

    frontLeftSparkMaxConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    frontLeftSparkMaxConfig.encoder
        .positionConversionFactor(kDriveEncoderRot2Meter)
        .velocityConversionFactor(kDriveEncoderRPM2MeterPerSec);
    backLeftSparkMaxConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .follow(m_LeftFrontMotor);
    backLeftSparkMaxConfig.encoder
        .positionConversionFactor(kDriveEncoderRot2Meter)
        .velocityConversionFactor(kDriveEncoderRPM2MeterPerSec);

    frontRightSparkMaxConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    frontRightSparkMaxConfig.encoder
        .positionConversionFactor(kDriveEncoderRot2Meter)
        .velocityConversionFactor(kDriveEncoderRPM2MeterPerSec);
    backRightSparkMaxConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .follow(m_RightFrontMotor);
    backRightSparkMaxConfig.encoder
        .positionConversionFactor(kDriveEncoderRot2Meter)
        .velocityConversionFactor(kDriveEncoderRPM2MeterPerSec);

    m_LeftFrontMotor.configure(frontLeftSparkMaxConfig, null, null);
    m_LeftBackMotor.configure(backLeftSparkMaxConfig, null, null);

    m_RightFrontMotor.configure(frontRightSparkMaxConfig, null, null);
    m_RightBackMotor.configure(backRightSparkMaxConfig, null, null);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

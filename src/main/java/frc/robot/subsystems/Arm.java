// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.revrobotics.AbsoluteEncoder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

public class Arm extends SubsystemBase {
    // The motors on the left side of the drive.
    private final SparkMax m_LeftFrontMotor = new SparkMax(57, MotorType.kBrushless);

    // The left-side drive encoder
    private AbsoluteEncoder m_LeftFrontEncoder = m_LeftFrontMotor.getAbsoluteEncoder();
    
        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
        private final MutVoltage m_appliedVoltage = Volts.mutable(0);
        // Mutable holder for unit-safe linear distance values, persisted to avoid
        // reallocation.
        private final MutAngle m_distance = Radians.mutable(0);
        // Mutable holder for unit-safe linear velocity values, persisted to avoid
        // reallocation.
        private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
        private final SparkMaxConfig frontLeftSparkMaxConfig = new SparkMaxConfig();
        private AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();
                
                // Create a new SysId routine for characterizing the drive.
                private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                // Tell SysId how to plumb the driving voltage to the motors.
                                (Voltage volts) -> {
                                    m_LeftFrontMotor.setVoltage(volts.in(Volts));
                                },
                                // Tell SysId how to record a frame of data for each motor on the mechanism
                                // being
                                // characterized.
                                log -> {
                                    // Record a frame for the left motors. Since these share an encoder, we consider
                                    // the entire group to be one motor.
                                    log.motor("elevator")
                                            .voltage(
                                                    m_appliedVoltage.mut_replace(
                                                            m_LeftFrontMotor.getAppliedOutput() * m_LeftFrontMotor.getBusVoltage(),
                                                            Volts))
                                            .angularPosition(m_distance.mut_replace(m_LeftFrontEncoder.getPosition()
                                                    , Radians))
                                            .angularVelocity(
                                                    m_velocity.mut_replace(
                                                            m_LeftFrontEncoder.getVelocity(),
                                                            RadiansPerSecond));
                                },
                                // Tell SysId to make generated commands require this subsystem, suffix test
                                // state in
                                // WPILog with this subsystem's name ("drive")
                                this));
            
                /** Creates a new Drive subsystem. */
      public Arm() {
        frontLeftSparkMaxConfig.inverted(true)
        .idleMode(IdleMode.kBrake);
            
        m_LeftFrontEncoder = m_LeftFrontMotor.getAbsoluteEncoder();
        config = new AbsoluteEncoderConfig();
        config.positionConversionFactor(Math.PI * 2)  
          .velocityConversionFactor(Math.PI / 30)
          .zeroOffset(0.312)
          .inverted(true);
      frontLeftSparkMaxConfig.apply(config);

      m_LeftFrontMotor.configure(frontLeftSparkMaxConfig, null, null);       

    }

    @Override
    public void periodic() {}

    /**
     * Returns a command that drives the robot with arcade controls. 
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public Command arcadeDriveCommand(DoubleSupplier fwd) {
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        return run(() -> m_LeftFrontMotor.set(fwd.getAsDouble()))
                .withName("arcadeDrive");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}

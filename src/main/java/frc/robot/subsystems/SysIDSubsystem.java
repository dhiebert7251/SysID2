// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
//import static edu.wpi.first.units.Units.Radians;
//import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.Seconds;


import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Time;




public class SysIDSubsystem extends SubsystemBase {
  /** Creates a new SysIDSubsystem. */

  private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(DriveConstants.kRearLeftMotorPort);
  private final WPI_TalonSRX m_frontRight = new WPI_TalonSRX(DriveConstants.kFrontRightMotorPort);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(DriveConstants.kRearRightMotorPort);

  //private final MecanumDrive m_drive =
  //  new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);
   
    private final ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain Data");

    private GenericEntry FLDistanceEntry;
    private GenericEntry RLDistanceEntry;
    private GenericEntry FRDistanceEntry;
    private GenericEntry RRDistanceEntry;

    private GenericEntry FLPositionEntry;
    private GenericEntry RLPositionEntry;
    private GenericEntry FRPositionEntry;
    private GenericEntry RRPositionEntry;


  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder =
    new Encoder(
      DriveConstants.kFrontLeftEncoderPorts[0],
      DriveConstants.kFrontLeftEncoderPorts[1],
      DriveConstants.kFrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
    new Encoder(
      DriveConstants.kRearLeftEncoderPorts[0],
      DriveConstants.kRearLeftEncoderPorts[1],
      DriveConstants.kRearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
    new Encoder(
      DriveConstants.kFrontRightEncoderPorts[0],
      DriveConstants.kFrontRightEncoderPorts[1],
      DriveConstants.kFrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
    new Encoder(
      DriveConstants.kRearRightEncoderPorts[0],
      DriveConstants.kRearRightEncoderPorts[1],
      DriveConstants.kRearRightEncoderReversed);

    //SysID
      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
      private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
      // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
      private final MutableMeasure<Distance> m_distance = mutable(Feet.of(0));
      // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
      private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(FeetPerSecond.of(0));


      // Create a new SysId routine for characterizing the drive.
      private final SysIdRoutine m_sysIdRoutine =
          new SysIdRoutine(
              // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                  // Tell SysId how to plumb the driving voltage to the motors.
                  (Measure<Voltage> volts) -> {
                    m_frontLeft.setVoltage(volts.in(Volts));
                    m_rearLeft.setVoltage(volts.in(Volts));
                    m_frontRight.setVoltage(volts.in(Volts));
                    m_rearRight.setVoltage(volts.in(Volts));
                  },
                  // Tell SysId how to record a frame of data for each motor on the mechanism being
                  // characterized.
                  log -> {
                    // Record a frame for the each motor
                    log.motor("frontLeft")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                m_frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(m_frontLeftEncoder.getDistance(), Feet))
                        .linearVelocity(
                            m_velocity.mut_replace(m_frontLeftEncoder.getRate(), FeetPerSecond));

                    log.motor("frontRight")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                m_frontRight.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(m_frontRightEncoder.getDistance(), Feet))
                        .linearVelocity(
                            m_velocity.mut_replace(m_rearRightEncoder.getRate(), FeetPerSecond));

                    log.motor("rearLeft")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                m_rearLeft.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(m_rearLeftEncoder.getDistance(), Feet))
                        .linearVelocity(
                            m_velocity.mut_replace(m_rearLeftEncoder.getRate(), FeetPerSecond));

                    log.motor("rearRight")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                m_rearRight.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(m_rearRightEncoder.getDistance(), Feet))
                        .linearVelocity(
                            m_velocity.mut_replace(m_rearRightEncoder.getRate(), FeetPerSecond));
                  },
                  // Tell SysId to make generated commands require this subsystem, suffix test state in
                  // WPILog with this subsystem's name ("SysIDSubsystem")
                  this));

                  





  public SysIDSubsystem() {
    

    //Testbed Factory reset motor controllers
    m_frontLeft.configFactoryDefault();
    m_rearLeft.configFactoryDefault();
    m_frontRight.configFactoryDefault();
    m_rearRight.configFactoryDefault();

    //Testbed break/coast mode
    m_frontLeft.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    m_rearLeft.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    m_frontRight.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    m_rearRight.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

    //Testbed
    m_frontLeftEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(PhysicalConstants.kEncoderDistancePerPulse);
    
    //Invert motors on one side
    m_frontLeft.setInverted(DriveConstants.kFrontLeftMotorReversed);
    m_rearLeft.setInverted(DriveConstants.kRearLeftMotorReversed);
    m_frontRight.setInverted(DriveConstants.kFrontRightMotorReversed);
    m_rearRight.setInverted(DriveConstants.kRearRightMotorReversed);

    //turn of motor safety check
    m_frontLeft.setSafetyEnabled(false);
    m_rearLeft.setSafetyEnabled(false);
    m_rearRight.setSafetyEnabled(false);
    m_frontRight.setSafetyEnabled(false);

    //reset encoders
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();

     // Initialize Shuffleboard widgets for encoder positions
    FLPositionEntry = driveTab.add("FL Encoder Position", 0).getEntry();
    RLPositionEntry = driveTab.add("RL Encoder Position", 0).getEntry();
    FRPositionEntry = driveTab.add("FR Encoder Position", 0).getEntry();
    RRPositionEntry = driveTab.add("RR Encoder Position", 0).getEntry();

    // Initialize Shuffleboard widgets for encoder distances
    FLDistanceEntry = driveTab.add("FL Encoder Distance", 0.0).getEntry();
    RLDistanceEntry = driveTab.add("RL Encoder Distance", 0.0).getEntry();
    FRDistanceEntry = driveTab.add("FR Encoder Distance", 0.0).getEntry();
    RRDistanceEntry = driveTab.add("RR Encoder Distance", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showTelemetry();
  }
  
  /*
  public Command arcadeDriveCommand(DoubleSupplier fwd, double strafe, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    //return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), 0, rot.getAsDouble()))
    //    .withName("arcadeDrive");
    return run(() -> m_drive.driveCartesian(fwd.getAsDouble(), 0, rot.getAsDouble()))
        .withName("Drive Cartesion");
  }
    */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  
  public Encoder getFrontLeftEncoder() {
    //Testbed
    return m_frontLeftEncoder;
  }

  public Encoder getRearLeftEncoder() {
    //Testbed
    return m_rearLeftEncoder;
  }

  public Encoder getFrontRightEncoder() {
    //Testbed
    return m_frontRightEncoder;
  }


  public Encoder getRearRightEncoder() {

    //Testbed
    return m_rearRightEncoder;
  }

  
  public void showTelemetry(){
    // Update the Shuffleboard entries with current values
    FLPositionEntry.setDouble(m_frontLeftEncoder.getRaw());
    RLPositionEntry.setDouble(m_rearLeftEncoder.getRaw());
    FRPositionEntry.setDouble(m_frontRightEncoder.getRaw());
    RRPositionEntry.setDouble(m_rearRightEncoder.getRaw());

    FLDistanceEntry.setDouble(m_frontLeftEncoder.getDistance());
    RLDistanceEntry.setDouble(m_rearLeftEncoder.getDistance());
    FRDistanceEntry.setDouble(m_frontRightEncoder.getDistance());
    RRDistanceEntry.setDouble(m_rearRightEncoder.getDistance());

   
  
  }
}

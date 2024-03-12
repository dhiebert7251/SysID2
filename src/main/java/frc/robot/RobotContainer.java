// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SysIDSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final SysIDSubsystem m_sysID = new SysIDSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  //PS4Controller m_driverController = new PS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", null);
    autoChooser.addOption("Quasi Fwd", m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Quasi Rev", m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Dynamic Fwd", m_sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Dynamic Rev", m_sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //SYS ID
    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    new JoystickButton(m_driverController, Button.kCross.value)
      .onTrue(new InstantCommand(() -> m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
    new JoystickButton(m_driverController, Button.kCircle.value)
      .onTrue(new InstantCommand(() -> m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
    new JoystickButton(m_driverController, Button.kSquare.value)
      .onTrue(new InstantCommand(() -> m_sysID.sysIdDynamic(SysIdRoutine.Direction.kForward)));
    new JoystickButton(m_driverController, Button.kTriangle.value)
      .onTrue(new InstantCommand(() -> m_sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  
  }
}

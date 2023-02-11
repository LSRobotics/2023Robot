// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    final DriveTrain.ArcadeDriveCommand drivetrain_command = m_DriveTrain.new ArcadeDriveCommand(
      () -> -m_driverController.getLeftY(),
      () -> m_driverController.getRightX()
    ); //technically the second argument can just be passed directly as a lambda (m_dirverController::getRightX), but it is kept as an inline lambda for symmetry
    drivetrain_command.addRequirements(m_DriveTrain);
    //Set the DriveTrain subsystem to automatically call the drive function by default
    m_DriveTrain.setDefaultCommand(
      //Accesses the command class ArcadeDriveCommand from within the instanced DriveTrain class.
      drivetrain_command
    );
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    //Toggle intake speed based on a threshold 
    //TODO: Add constants for the trigger thresholds
    Trigger leftTriggerToggle = new Trigger(() -> {
      return m_operatorController.getLeftTriggerAxis() > 0.5;
    });
    Trigger rightTriggerToggle = new Trigger(() -> {
      return m_operatorController.getRightTriggerAxis() > 0.5;
    });
    rightTriggerToggle
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPowerScalar(Constants.IntakeConstants.intake_fast_speed)));
    leftTriggerToggle
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPowerScalar(Constants.IntakeConstants.intake_slow_speed)));
    
    //reset intake speed when both triggers are not being held
    leftTriggerToggle.and(rightTriggerToggle).onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setPowerScalar(Constants.IntakeConstants.intake_default_speed)));

    //Toggle intake power based on right and left bumper
    m_operatorController.rightBumper()
        .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(.3)));
    m_operatorController.leftBumper()
        .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(-.3)));
    
    //disable intake when both buttons are false
    m_operatorController.a().and(m_operatorController.b()).onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setPower(0.0)));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

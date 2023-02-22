// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
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
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    final DriveTrain.ArcadeDriveCommand drivetrain_command = m_DriveTrain.new ArcadeDriveCommand(
      () -> (m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()),
      () -> m_driverController.getLeftX()
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



   
    
    m_operatorController.leftTrigger() //intake slow
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(Constants.IntakeConstants.intake_slow_speed)));
    m_operatorController.rightTrigger() //intake fast
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(Constants.IntakeConstants.intake_fast_speed)));
    m_operatorController.rightBumper() //outtake (shoot object out of intake)
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(-Constants.IntakeConstants.intake_fast_speed)));

    m_operatorController.rightTrigger().or(m_driverController.leftTrigger()).or(m_operatorController.rightBumper()).onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setPower(0.0)));

    m_operatorController.a().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setArmSpeed(1);
      System.out.println("BAZINGA");
    }));
    m_operatorController.b().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setArmSpeed(-1);
      System.out.println("BAZLOOPER");
    }));

    m_operatorController.a().or(m_operatorController.b()).onFalse(Commands.runOnce(() -> {m_ArmSubsystem.setArmSpeed(0);}));

    // m_operatorController.leftTrigger().whileTrue(Commands.startEnd( () -> m_IntakeSubsystem.setPower(.3), () -> m_IntakeSubsystem.setPower(0.0)));
    // m_operatorController.rightTrigger().whileTrue(Commands.startEnd( () -> m_IntakeSubsystem.setPower(.5), () -> m_IntakeSubsystem.setPower(0.0)));
    // m_operatorController.rightBumper().whileTrue(Commands.startEnd( () -> m_IntakeSubsystem.setPower(-.5), () -> m_IntakeSubsystem.setPower(0.0)));

    //m_operatorController.leftBumper().whileTrue(Commands.startEnd( () -> m_ArmSubsystem.setArmSpeed(0.2,0.2), () -> m_ArmSubsystem.setArmSpeed(0.0, 0.0)));
    //m_operatorController.leftBumper().whileTrue(Commands.startEnd( () -> m_ArmSubsystem.setArmPIDAngles(90,90), () -> m_ArmSubsystem.setArmPIDAngles(0.0, 0.0)));

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
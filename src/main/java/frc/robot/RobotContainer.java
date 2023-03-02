// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.VisionAlign;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.HashMap;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();

  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
 
  private SlewRateLimiter filter = new SlewRateLimiter(2.8);
  private SlewRateLimiter filter2 = new SlewRateLimiter(2.8);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(1);

  private boolean cubeMode = false;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private HashMap<String, CommandBase> AutonList = new HashMap<String, CommandBase>();


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    final DriveTrain.ArcadeDriveCommand drivetrain_command = m_DriveTrain.new ArcadeDriveCommand(
      () -> {
        return filter.calculate(.7*(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()));
      },
      () -> {return filter2.calculate(.5*m_driverController.getLeftX());}
    ); //technically the second argument can just be passed directly as a lambda (m_dirverController::getRightX), but it is kept as an inline lambda for symmetry
    drivetrain_command.addRequirements(m_DriveTrain);
    //Set the DriveTrain subsystem to automatically call the drive function by default
    m_DriveTrain.setDefaultCommand(
      //Accesses the command class ArcadeDriveCommand from within the instanced DriveTrain class.
      drivetrain_command
    );

    autonListInit();

    dashboardInit();
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
    
    m_driverController.a().toggleOnTrue(new AutoBalance(m_DriveTrain));
    m_driverController.x()
      .toggleOnTrue(new VisionAlign(m_DriveTrain, m_VisionSubsystem));

    m_operatorController.rightTrigger().onTrue(Commands.runOnce(() -> 
        m_IntakeSubsystem.setPower(cubeMode ? Constants.IntakeConstants.CubeMode.intake_speed : Constants.IntakeConstants.ConeMode.intake_speed)
      )
    );
    m_operatorController.y() //outtake slow
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(
        cubeMode ? Constants.IntakeConstants.CubeMode.slow_outtake_speed : Constants.IntakeConstants.ConeMode.slow_outtake_speed
      )
    ));
    m_operatorController.b() //outtake fast
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setPower(
        cubeMode ? Constants.IntakeConstants.CubeMode.fast_outtake_speed : Constants.IntakeConstants.ConeMode.fast_outtake_speed
      )
    ));

    m_operatorController.rightTrigger()
    .or(m_operatorController.y())
    .or(m_operatorController.b()).onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setPower(0.0)));
    
    m_operatorController.povUp().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setArmSpeed(.4);
      System.out.println("BAZINGA");
    }));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setArmSpeed(-.4);
      System.out.println("BAZLOOPER");
    }));
    m_operatorController.povLeft().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setArmSpeed(-.2);
      System.out.println("BAZOOKA");
    }));
    m_operatorController.povRight().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setArmSpeed(.2);
      System.out.println("BAZAMA");
    }));

    m_operatorController.povUp().or(m_operatorController.povDown()).onFalse(Commands.runOnce(() -> {m_ArmSubsystem.setArmSpeed(0);}));
    m_operatorController.povRight().or(m_operatorController.povLeft()).onFalse(Commands.runOnce(() -> {m_ArmSubsystem.setArmSpeed(0);}));

    m_operatorController.a().onTrue(Commands.runOnce(() -> 
    {
      cubeMode = !cubeMode;
      m_LedSubsystem.setMode(cubeMode);
    }));
  }

  //PUT ALL AUTONS IN THIS AND IT WILL JUST WORK
  public void autonListInit() {
    //TODO: Please fix these autons
    //Add any subsequent autons here:
    AutonList.put("Auton with Balance", Autos.placeAndBalanceAuto(m_DriveTrain, m_IntakeSubsystem, m_ArmSubsystem));



    //This is the default auto
    //PLEASE DO NOT ADD ANOTHER AUTON AFTER THIS
    AutonList.put("Auton without Balance", Autos.placeAndExitAuton(m_DriveTrain, m_IntakeSubsystem, m_ArmSubsystem));
  }

  public void dashboardInit() {
    //Initialize what autonomous mode on the dashboard corresponds to which auton
    SmartDashboard.putData(CommandScheduler.getInstance()); //Displays scheduler status

    //Iterates over all keys in the Auton List and puts them on the smart dashboard
    for( String key : AutonList.keySet()){
      //Need to use setDefaultOption instead of addOption so that the last option is used as the default
      m_chooser.setDefaultOption(key, key);
    }
    SmartDashboard.putData("Auto choices", m_chooser);
  }

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutonList.get(m_chooser.getSelected());
  }
}
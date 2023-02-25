package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.Autos;



import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.DriveTrainConstants;


/** An example command that uses an example subsystem. */
public class VisionAlign extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final VisionSubsystem m_VisionSubsystem; 
  private final DriveTrain m_DriveTrainSubsystem;
  private boolean visionRight = false;
  private boolean visionLeft = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionAlign(DriveTrain DriveTrainSubsystem, VisionSubsystem VisionSubsystem) {
    m_DriveTrainSubsystem = DriveTrainSubsystem;
    m_VisionSubsystem = VisionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrainSubsystem, m_VisionSubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_VisionSubsystem.getTx() >= 5){
      visionLeft = true;
    }
    else if(m_VisionSubsystem.getTx()<= 5){
      visionRight = true;
    }
    else{
      visionRight = false;
      visionLeft = false;
    }

    /*  if(visionRight){
      drive_controller.arcadeDrive(0,.2);
     } */
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
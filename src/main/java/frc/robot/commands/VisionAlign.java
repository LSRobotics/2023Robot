package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;



/** An example command that uses an example subsystem. */
public class VisionAlign extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final VisionSubsystem m_VisionSubsystem; 
  private final DriveTrain m_DriveTrainSubsystem;
  private boolean visionRight;
  private boolean visionLeft;


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
    if(m_VisionSubsystem.getTx() <= -7){
      visionLeft = true;
      visionRight = false;
    }
    else if(m_VisionSubsystem.getTx()>= 7){
      visionRight = true;
      visionLeft = false;
    }
    else{
      visionRight = true;
      visionLeft = false;
    }

    double speed = 0;
    if(visionRight){
      speed = .3;
    } 
    else if (visionLeft) {
      speed = -.3;
    }
    m_DriveTrainSubsystem.arcadeDrive(0, speed);

  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrainSubsystem.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(visionRight || visionLeft);
  }
}
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

//This is the command for handling ArcadeDrive logic externally. It is located within this subsystem because only this subsystem accesses it.
public class ArcadeDriveCommand extends CommandBase {

    //These need to be lambdas (DoubleSupplier functions) because they need to be called every loop, returning different values each time
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier turnSpeed;

    private DriveTrain drive_controller;

    public ArcadeDriveCommand(DoubleSupplier speed, DoubleSupplier turn, DriveTrain drive_train) {
        driveSpeed = speed;
        turnSpeed = turn;
        drive_controller = drive_train;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      //This is only able to directly access drive_controller becuase it is a nested class
      //NOTE: if this code is changed to no longer be a nested class (for accessing another subsystem or something), this will no longer be able to directly access the drive (you just need to pass the subsystem and write an accessor method then)
      drive_controller.arcadeDrive(driveSpeed.getAsDouble(), turnSpeed.getAsDouble());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      drive_controller.arcadeDrive(0, 0);
    }
  }

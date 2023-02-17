// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax fl_motor = new CANSparkMax(MotorConstants.fl_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax fr_motor = new CANSparkMax(MotorConstants.fr_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax br_motor = new CANSparkMax(MotorConstants.br_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax bl_motor = new CANSparkMax(MotorConstants.bl_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);

    private MotorControllerGroup left_motors  = new MotorControllerGroup(fl_motor, bl_motor);
    private MotorControllerGroup right_motors = new MotorControllerGroup(fr_motor, br_motor);

    private DifferentialDrive drive_controller = new DifferentialDrive(right_motors, left_motors);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    super();

    //The loopRampRate could be made a constant
    fl_motor.setOpenLoopRampRate(0.5);
    fr_motor.setOpenLoopRampRate(0.5);
    br_motor.setOpenLoopRampRate(0.5);
    bl_motor.setOpenLoopRampRate(0.5);

    right_motors.setInverted(true);
    fl_motor.setIdleMode(IdleMode.kBrake);
    fr_motor.setIdleMode(IdleMode.kBrake);
    br_motor.setIdleMode(IdleMode.kBrake);
    bl_motor.setIdleMode(IdleMode.kBrake);

  }

  //This is the command for handling ArcadeDrive logic externally. It is located within this subsystem because only this subsystem accesses it.
  public class ArcadeDriveCommand extends CommandBase {

    //These need to be lambdas (DoubleSupplier functions) because they need to be called every loop, returning different values each time
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier turnSpeed;

    public ArcadeDriveCommand(DoubleSupplier speed, DoubleSupplier turn) {
        driveSpeed = speed;
        turnSpeed = turn;
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
  
}

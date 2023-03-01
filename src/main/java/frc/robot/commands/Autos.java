// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoBalance;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public final static CommandBase testAuto(DriveTrain driveTrain) {
    return Commands.sequence(
      new moveUntilAngled(driveTrain, -1)
      //new AutoBalance(driveTrain)
    );
  }

  public final static CommandBase placeAndBalanceAuto(DriveTrain driveTrain, IntakeSubsystem intakeSubsystem) {
    return Commands.sequence(
      Commands.race(
        Commands.startEnd(() -> {intakeSubsystem.setPower(IntakeConstants.ConeMode.fast_outtake_speed);}, () -> {intakeSubsystem.setPower(0);}, intakeSubsystem),
        new WaitCommand(0.5)
      )
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

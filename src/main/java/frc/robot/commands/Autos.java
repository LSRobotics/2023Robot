// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

  public final static CommandBase placeAndBalanceAuto(DriveTrain driveTrain, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, LEDSubsystem ledSubsystem) {
    return Commands.sequence(
      Commands.race(
        Commands.startEnd(() -> {armSubsystem.setArmSpeed(ArmConstants.fast_speed);}, () -> {armSubsystem.setArmSpeed(0);}),
        new WaitCommand(1.5)
      ),
      new WaitCommand(0.5),
      Commands.race(
        Commands.startEnd(() -> {intakeSubsystem.setPower(-0.7);}, () -> {intakeSubsystem.setPower(0);}, intakeSubsystem),
        new WaitCommand(0.5)
      ),
      //Move and retract arm
      Commands.parallel(
        new moveUntilAngled(driveTrain, -1),
        Commands.race(
          Commands.startEnd(() -> {armSubsystem.setArmSpeed(-ArmConstants.fast_speed);}, () -> {armSubsystem.setArmSpeed(0);}),
          new WaitCommand(2)
        )
      ),
      new AutoBalance(driveTrain),
      new PrintCommand("finished"),
      new StayStill(driveTrain, ledSubsystem)
    );
  }


  public final static CommandBase placeCone(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(
      Commands.race(
        Commands.startEnd(() -> {armSubsystem.setArmSpeed(ArmConstants.fast_speed);}, () -> {armSubsystem.setArmSpeed(0);}),
        new WaitCommand(1.5)
      ),
      new WaitCommand(0.5),
      Commands.race(
        Commands.startEnd(() -> {intakeSubsystem.setPower(-1);}, () -> {intakeSubsystem.setPower(0);}, intakeSubsystem),
        new WaitCommand(0.5)
      )
      
    );
      
    
  }

  public final static CommandBase placeAndExitAuton(DriveTrain driveTrain, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(
      Commands.race(
        Commands.startEnd(() -> {armSubsystem.setArmSpeed(ArmConstants.fast_speed);}, () -> {armSubsystem.setArmSpeed(0);}),
        new WaitCommand(1.5)
      ),
      new WaitCommand(0.5),
      Commands.race(
        Commands.startEnd(() -> {intakeSubsystem.setPower(-0.7);}, () -> {intakeSubsystem.setPower(0);}, intakeSubsystem),
        new WaitCommand(0.5)
      ),
      new pidDrive(-180, driveTrain)
    );
  }

  public final static CommandBase testLeavePlaceAndBalanceAuto(DriveTrain driveTrain, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, LEDSubsystem ledSubsystem) {
    return Commands.sequence(
      Commands.race(
        Commands.startEnd(() -> {armSubsystem.setArmSpeed(ArmConstants.fast_speed);}, () -> {armSubsystem.setArmSpeed(0);}),
        new WaitCommand(1.5)
      ),
      new WaitCommand(0.5),
      Commands.race(
        Commands.startEnd(() -> {intakeSubsystem.setPower(-0.7);}, () -> {intakeSubsystem.setPower(0);}, intakeSubsystem),
        new WaitCommand(0.5)
      ),
      //Move and retract arm
      Commands.parallel(
        new pidDrive(-168, driveTrain),
        Commands.race(
          Commands.startEnd(() -> {armSubsystem.setArmSpeed(-ArmConstants.fast_speed);}, () -> {armSubsystem.setArmSpeed(0);}),
          new WaitCommand(2)
        )
      ),
      new moveUntilAngled(driveTrain, 1),
      new AutoBalance(driveTrain),
      new PrintCommand("finished"),
      new StayStill(driveTrain, ledSubsystem)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

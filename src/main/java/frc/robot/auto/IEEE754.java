// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.commands.drive.CyborgCommandDriveDistance;
import frc.robot.commands.flywheel.InstantCommandIdleFlywheel;
import frc.robot.commands.flywheel.InstantCommandToggleFlywheel;
import frc.robot.commands.intake.CyborgCommandEnablePrimaryIntake;
import frc.robot.commands.intake.CyborgCommandEnableSecondaryIntake;
import frc.robot.commands.intake.CyborgCommandToggleAdjust;
import frc.robot.commands.misc.CyborgCommandWait;
import frc.robot.commands.turret.CyborgCommandSetTurretPosition;
import frc.robot.commands.turret.CyborgCommandSetYawPosition;
import frc.robot.commands.turret.CyborgCommandZeroTurret;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.subsystems.SubsystemVision;

public class IEEE754 {

  private final Command auto;

  public IEEE754(SubsystemDrive drivetrain, SubsystemFlywheel flywheel, SubsystemTurret turret,
      SubsystemIntake intake, SubsystemVision vision) {
    // Zero turret
    // Set turret to -51 rotations
    // Lower adjust
    // Enable pimary intake
    // Drive 60 inches
    // Enable flywheel vision PID
    // Enable turret vision PID
    // wait 2 seconds
    // enable secondary intake
    // disable primary intake
    // disable secondary intake
    // disable turret vision PID
    // Enable flywheel idle
    Command zeroTurret = new CyborgCommandZeroTurret(turret);
    Command centerTurret = new CyborgCommandSetYawPosition(turret, -53D).withTimeout(2D);
    Command lowerAdjust = new CyborgCommandToggleAdjust(intake);
    Command enablePrimaryIntake = new CyborgCommandEnablePrimaryIntake(intake);
    Command driveForward = new CyborgCommandDriveDistance(drivetrain, 80D);
    Command driveBackward = new CyborgCommandDriveDistance(drivetrain, -5D);
    Command pickupCargo = new CyborgCommandWait(1000);
    Command enableFlywheelIdle = new InstantCommandToggleFlywheel(flywheel);
    Command enableFlywheelPID = new InstantCommandIdleFlywheel(flywheel);
    Command enableTurretPID = new CyborgCommandSetTurretPosition(turret);
    Command steadyTurret = new CyborgCommandWait(3000);
    Command enableSecondayIntake = new CyborgCommandEnableSecondaryIntake(intake, 1D);

    Command preTaxi = lowerAdjust.alongWith(enableFlywheelIdle, zeroTurret.andThen(centerTurret));
    Command visionInit = enableFlywheelPID.alongWith(enableTurretPID);
    Command ShootBalls = steadyTurret.andThen(enableSecondayIntake);
    this.auto = preTaxi
        .andThen(enablePrimaryIntake
            .alongWith(driveForward.andThen(pickupCargo, driveBackward, visionInit.alongWith(ShootBalls))));
  }

  public Command getAutonomousCommand() {
    return auto.withTimeout(15D);
  }
}

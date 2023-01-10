package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.CyborgCommandDriveDistance;
import frc.robot.commands.flywheel.InstantCommandIdleFlywheel;
import frc.robot.commands.flywheel.InstantCommandToggleFlywheel;
import frc.robot.commands.intake.CyborgCommandEnablePrimaryIntake;
import frc.robot.commands.intake.CyborgCommandEnableSecondaryIntake;
import frc.robot.commands.intake.CyborgCommandToggleAdjust;
import frc.robot.commands.turret.CyborgCommandSetTurretPosition;
import frc.robot.commands.turret.CyborgCommandSetTurretPositionManual;
import frc.robot.commands.turret.CyborgCommandZeroTurret;
import frc.robot.commands.misc.CyborgCommandWait;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.subsystems.SubsystemUltrasonic;
import frc.robot.subsystems.SubsystemVision;

public class TerminalAuto implements AutonomousCommand {
        private final Command autonomousCommand;

        public TerminalAuto(SubsystemDrive drivetrain, SubsystemFlywheel flywheel, SubsystemIntake intake,
                        SubsystemTurret turret, SubsystemUltrasonic ultrasonic, SubsystemVision vision) {
                Command zeroTurret = new CyborgCommandZeroTurret(turret);
                Command lowerAdjust = new CyborgCommandToggleAdjust(intake);
                Command enablePrimaryIntake = new CyborgCommandEnablePrimaryIntake(intake);
                Command driveForward = new CyborgCommandDriveDistance(drivetrain, 75D);
                Command enableFlywheelIdle = new InstantCommandToggleFlywheel(flywheel);
                Command toggleFlywheelPID = new InstantCommandIdleFlywheel(flywheel);
                Command enableTurretPID = new CyborgCommandSetTurretPosition(turret);
                Command steadyTurret0 = new CyborgCommandWait(500);
                Command waitHalfSecond = new CyborgCommandWait(500);
                Command waitHalfSecond2 = new CyborgCommandWait(750);
                Command firstBallTurretPosition = new CyborgCommandSetTurretPositionManual(turret, -47D, 145D, 3650D);
                Command enableSecondayIntake0 = new CyborgCommandEnableSecondaryIntake(intake, 0.61D);
                Command enableSecondayIntake1 = new CyborgCommandEnableSecondaryIntake(intake, 1D);
                // Command driveToWall = new CyborgCommandDriveUsingUltrasonic(drivetrain,
                // ultrasonic, 24D);
                Command driveToWall = new CyborgCommandDriveDistance(drivetrain, 182D);
                Command collectCargo = new CyborgCommandEnablePrimaryIntake(intake).withTimeout(2D);
                Command driveBackwards = new CyborgCommandDriveDistance(drivetrain, -150D);

                Command firstTaxi = lowerAdjust
                                .alongWith(enableFlywheelIdle, waitHalfSecond.andThen(driveForward),
                                                enablePrimaryIntake, toggleFlywheelPID, zeroTurret
                                                                .andThen(firstBallTurretPosition))
                                .withTimeout(3D);
                Command shootCargo0 = steadyTurret0.andThen(enableSecondayIntake0);
                Command shootCargo1 = waitHalfSecond2.andThen(enableSecondayIntake1);
                Command cycleCargo0 = shootCargo0.withTimeout(2.5D);
                Command secondTaxi = driveToWall.andThen(collectCargo, driveBackwards);
                this.autonomousCommand = firstTaxi
                                .andThen(cycleCargo0.andThen(secondTaxi, enableTurretPID
                                .alongWith(shootCargo1)));

        }

        @Override
        public Command getAutonomousCommand() {
                return autonomousCommand;
        }

        @Override
        public boolean requiresFlywheel() {
                return true;
        }

}

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.CyborgCommandDriveDistance;
import frc.robot.commands.flywheel.InstantCommandIdleFlywheel;
import frc.robot.commands.flywheel.InstantCommandToggleFlywheel;
import frc.robot.commands.intake.CyborgCommandEnablePrimaryIntake;
import frc.robot.commands.intake.CyborgCommandEnableSecondaryIntake;
import frc.robot.commands.intake.CyborgCommandToggleAdjust;
import frc.robot.commands.misc.CyborgCommandWait;
import frc.robot.commands.turret.CyborgCommandSetTurretPosition;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.subsystems.SubsystemVision;

public class IEEE754Auto implements AutonomousCommand {

    private Command lowerAdjust, enableIntake, enableFullIntake, driveForward, startTurretPID,
            startFlywheelPID, wait, waitForPID;

    public IEEE754Auto(SubsystemDrive drivetrain, SubsystemFlywheel flywheel,
            SubsystemTurret turret, SubsystemIntake intake, SubsystemVision vision) {
        this.driveForward = new CyborgCommandDriveDistance(drivetrain, 70D);
        this.lowerAdjust = new CyborgCommandToggleAdjust(intake);
        this.enableIntake = new CyborgCommandEnablePrimaryIntake(intake);
        this.enableFullIntake = new CyborgCommandEnableSecondaryIntake(intake, 1D);
        this.startTurretPID = new CyborgCommandSetTurretPosition(turret);
        this.startFlywheelPID = new InstantCommandToggleFlywheel(flywheel)
                .alongWith(new InstantCommandIdleFlywheel(flywheel));
        this.wait = new CyborgCommandWait(2000);
        this.waitForPID = new CyborgCommandWait(4000);

    }

    @Override
    public Command getAutonomousCommand() {
        Command intakeInit = lowerAdjust;
        Command driveAndCollect = intakeInit.andThen(enableIntake.alongWith(driveForward));
        Command enableTurretPIDAndShoot = startTurretPID.alongWith(waitForPID.andThen(enableFullIntake));
        Command turretPID = startFlywheelPID
                .alongWith(wait.andThen(enableTurretPIDAndShoot));
        return driveAndCollect.alongWith(turretPID);// .withTimeout(9D);
    }

    @Override
    public boolean requiresFlywheel() {
        return true;
    }

}

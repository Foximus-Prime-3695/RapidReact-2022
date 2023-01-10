// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;

import static java.lang.Math.E;
import static java.lang.Math.pow;

public class CyborgCommandDriveDistanceLogistic extends CommandBase {
    SubsystemDrive drivetrain;
    double targetRotations;
    private double leftMasterDestination, rightMasterDestination;
    private int currentSchedule;
    private double allowedError;

    /** Creates a new CyborgCommandDriveDistance. */
    public CyborgCommandDriveDistanceLogistic(SubsystemDrive drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.targetRotations = distance * Constants.DRIVE_ROTATIONS_PER_INCH;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.leftMasterDestination = drivetrain.getLeftMasterPosition() + targetRotations;
        this.rightMasterDestination = drivetrain.getRightMasterPosition() + targetRotations;

        // double p = Util.getAndSetDouble("Drivetrain kP", 0.03);
        // double i = Util.getAndSetDouble("Drivetrain kI", 0);
        // double d = Util.getAndSetDouble("Drivetrain kD", 0);
        // double f = Util.getAndSetDouble("Drivetrain kF", 0);
        // double iZone = Util.getAndSetDouble("Drivetrain IZone", 0);
        // double out = Constants.AUTONOMOUS_INHIBITOR;
        // drivetrain.setPIDConstants(p, i, d, f, iZone, out);

        this.currentSchedule = 0;
        this.allowedError = Constants.DRIVETRAIN_ALLOWABLE_ERROR * Constants.DRIVE_ROTATIONS_PER_INCH;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.currentSchedule++;

        double speed = Constants.AUTONOMOUS_DRIVE_INHIBITOR / (1 + pow(E, -0.5D * (currentSchedule - 10)));
        if (leftMasterDestination - drivetrain.getLeftMasterPosition() > allowedError)
            drivetrain.setLeftSpeed(speed);
        if (rightMasterDestination - drivetrain.getRightMasterPosition() > allowedError)
            drivetrain.setRightSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setLeftSpeed(0D);
        drivetrain.setRightSpeed(0D);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double leftError = Math.abs(leftMasterDestination - drivetrain.getLeftMasterPosition());
        double rightError = Math.abs(rightMasterDestination - drivetrain.getRightMasterPosition());

        boolean rightWithinRange = rightError < allowedError;
        boolean leftWithinRange = leftError < allowedError;

        return rightWithinRange && leftWithinRange;
    }
}

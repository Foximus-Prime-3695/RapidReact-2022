package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.flywheel.ManualCommandFlywheel;
import frc.robot.util.Util;

public class SubsystemFlywheel extends SubsystemBase {

    CANSparkMax turretFlywheel;
    private boolean isDisabled = true;
    private boolean isIdle = true;

    /** Creates a new SubsystemBallTransport. */
    public SubsystemFlywheel() {
        this.turretFlywheel = new CANSparkMax(Constants.FLYWHEEL_ID, MotorType.kBrushless);

        turretFlywheel.setIdleMode(IdleMode.kCoast);
        turretFlywheel.setInverted(false);
        setDefaultCommand(new ManualCommandFlywheel(this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current flywheel RPM", turretFlywheel.getEncoder().getVelocity() * 1.6071D);
        SmartDashboard.putNumber("Flywheel Bus Voltage", turretFlywheel.getBusVoltage());
        SmartDashboard.putNumber("Flywheel Supply Current", turretFlywheel.getOutputCurrent());
        SmartDashboard.putBoolean("Flywheel Enabled", !isIdle);
    }

    public void setPIDF(double p, double i, double d, double f, double izone, double lowLimit, double highLimit) {
        turretFlywheel.getPIDController().setP(p, 0);
        turretFlywheel.getPIDController().setI(i, 0);
        turretFlywheel.getPIDController().setD(d, 0);
        turretFlywheel.getPIDController().setFF(f, 0);
        turretFlywheel.getPIDController().setIZone(izone, 0);
        turretFlywheel.getPIDController().setOutputRange(lowLimit, highLimit);
    }

    /**
     * Sets the target velocity (RPM) of the MOTOR, NOT the flywheel
     * 
     * @param velocity
     */
    public void setVelocity(double velocity) {
        double idleVelocity = Util.getAndSetDouble("Idle Velocity", 2000D);
        //idleVelocity = Math.abs(idleVelocity) > 5000 ? 5000 : Math.abs(idleVelocity);
        if (isDisabled)
            setPercentOutput(0D);
        else {
            if (isIdle) {
                if (turretFlywheel.getEncoder().getVelocity() * 1.6071D > idleVelocity + 50D)
                    setPercentOutput(0D);
                else
                    turretFlywheel.getPIDController().setReference(idleVelocity / 1.6071D, ControlType.kVelocity);
            } else
                turretFlywheel.getPIDController().setReference(velocity, ControlType.kVelocity);
        }
    }

    public void setPercentOutput(double output) {
        turretFlywheel.set(output);
    }

    // TODO: rename to imply only pid values
    public void toggleDisable() {
        this.isDisabled = !this.isDisabled;
    }

    public void toggleIdle() {
        this.isIdle = !this.isIdle;
    }

}

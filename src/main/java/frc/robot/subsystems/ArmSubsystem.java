package frc.robot.subsystems;
import frc.robot.consts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityDutyCycle;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX motorArm = new TalonFX(consts.CANID.armCanID);
    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
    public double armAngle; // Current angle of the arm in degrees
    private double targetPosition = 0;
    private double lastKP = consts.ArmPID.armKP.get();
    private double lastKI = consts.ArmPID.armKI.get();
    private double lastKD = consts.ArmPID.armKD.get();
    private TalonFXConfiguration config = new TalonFXConfiguration();
    public ArmSubsystem() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.withSlot0(new Slot0Configs()
            .withKP(consts.ArmPID.armKP.get())
            .withKI(consts.ArmPID.armKI.get())
            .withKD(consts.ArmPID.armKD.get()));
        motorArm.getConfigurator().apply(config);
    }

    /**
     * Resets the current position of the arm to 0° (sets encoder to zero).
     */
    public void resetArmPositionToZero() {
        motorArm.setPosition(0);
        Logger.recordOutput("Arm/EncoderZeroed", true);
    }

    /**
     * Sets the arm position to a desired angle.
     * @param targetAngle The desired angle in degrees.
     */
    public void setArmPosition(double targetAngle) {
        targetPosition = targetAngle;
        motorArm.setControl(new PositionDutyCycle(targetAngle));
    }

    public void armRPMControl(double rpm) {
        motorArm.setControl(velocityRequest.withVelocity(rpm));
    }

    @Override
    public void periodic() {
        // Update PID if changed
        if (consts.ArmPID.armKP.get() != lastKP || consts.ArmPID.armKI.get() != lastKI || consts.ArmPID.armKD.get() != lastKD) {
            config.Slot0.kP = consts.ArmPID.armKP.get();
            config.Slot0.kI = consts.ArmPID.armKI.get();
            config.Slot0.kD = consts.ArmPID.armKD.get();
            lastKP = consts.ArmPID.armKP.get();
            lastKI = consts.ArmPID.armKI.get();
            lastKD = consts.ArmPID.armKD.get();
        }
        // Log the current arm position every cycle
        Logger.recordOutput("Arm/Position", motorArm.getPosition().getValue());
        Logger.recordOutput("Arm/TargetAngle", targetPosition);
    }
}
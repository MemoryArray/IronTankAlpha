package frc.robot.subsystems;

import frc.robot.consts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX motorArm = new TalonFX(consts.CANID.armCanID);
    private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private double curPos = motorArm.getPosition().getValueAsDouble();
    private double desiredPosition = curPos;

    private Slot0Configs slot0 = armConfig.Slot0;
    private boolean isStalled = false;
    private int stallCount = 0;

    public ArmSubsystem() {
        armConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.armMaxCurrent;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.armMaxCurrent;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        slot0.kP = consts.ArmPID.armKP.get();
        slot0.kI = consts.ArmPID.armKI.get();
        slot0.kD = consts.ArmPID.armKD.get();
        motorArm.getConfigurator().apply(armConfig);
    }

    public void failsafe() {
        double current = motorArm.getSupplyCurrent().getValueAsDouble();
        double velocity = motorArm.getVelocity().getValueAsDouble();
        if (current > consts.Limits.armStallCurrent && velocity < consts.Limits.armStallThreshold) {
            if (stallCount > 10) {
                stall();
            }
            else {
                stallCount++;
            }
        }
    }

    public void stall() {
        isStalled = true;
        motorArm.set(0); // Stop the arm motor when stalled
    }

    public void clearStall() {
        isStalled = false;
        stallCount = 0;
    }

    public void setPosition(double position) {
        desiredPosition = position;
        motorArm.setControl(positionRequest.withPosition(desiredPosition));
    }

    public void rotate(double rotations) {
        desiredPosition = curPos + rotations;
        setPosition(desiredPosition);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Position", motorArm.getPosition().getValueAsDouble());
        Logger.recordOutput("Arm/Velocity", motorArm.getVelocity().getValueAsDouble());
        Logger.recordOutput("Arm/Voltage", motorArm.getSupplyVoltage().getValueAsDouble());
        Logger.recordOutput("Arm/Current", motorArm.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Arm/Stalled", isStalled);
        Logger.recordOutput("Arm/DesiredPosition", desiredPosition);

        failsafe();
        if (isStalled) stall();
        curPos = motorArm.getPosition().getValueAsDouble();
    }
}
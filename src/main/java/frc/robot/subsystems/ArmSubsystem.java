package frc.robot.subsystems;
import frc.robot.consts;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX motorArm = new TalonFX(consts.CANID.armCanIDci);
    public Angle armAngle; // Current angle of the arm in degrees

    public ArmSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorArm.getConfigurator().apply(config);
    }

    /**
     * Resets the current position of the arm to 0°.
     */
    public void resetArmPosition() {
        // Reset the encoder position to zero
        armAngle = motorArm.getPosition().getValue(); // Extract angle in degrees
    }
    public void resetArmPositionToZero() {
        motorArm.setPosition(armAngle); // Reset the motor position 
    }
    /**
     * Sets the arm position to a desired angle.
     * @param targetAngle The desired angle in degrees.
     */
    public void setArmPosition(double targetAngle) {
        // Convert the target angle to motor position units (if necessary)
        double targetPosition = targetAngle; // Assuming 1 degree corresponds to 1 unit
        motorArm.setPosition(targetPosition); // Set the motor position
    }
}
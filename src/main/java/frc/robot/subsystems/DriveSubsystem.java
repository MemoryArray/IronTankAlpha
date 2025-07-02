package frc.robot.subsystems;

import frc.robot.consts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable pidTable = ntInstance.getTable("PID");
  private final TalonFX motorFrontRight = new TalonFX(consts.CANID.RCanIDci);
  private final TalonFX motorFrontLeft = new TalonFX(consts.CANID.LCanIDci);
  private final TalonFXConfiguration configLeft = genConfig(true);
  private final TalonFXConfiguration configRight = genConfig(false);
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

  /** Enum to track which PID mode is active */
  private enum DriveMode {
    VELOCITY, POSITION
  }

  private DriveMode currentMode = null;
  private double desiredLeftRPM = 0.0;
  private double desiredRightRPM = 0.0;
  
  public DriveSubsystem() {
    motorFrontLeft.getConfigurator().apply(configLeft);
    motorFrontRight.getConfigurator().apply(configRight);
  }

  /** Generates a TalonFXConfiguration with the given inversion setting */
  private TalonFXConfiguration genConfig(boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    
    double velKPcd = pidTable.getEntry("VelPID/velKPcd").getDouble(0.0);
    double velKIcd = pidTable.getEntry("VelPID/velKIcd").getDouble(0.0);
    double velKDcd = pidTable.getEntry("VelPID/velKDcd").getDouble(0.0);

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = velKPcd;
    slot0.kI = velKIcd;
    slot0.kD = velKDcd;
    
    return config;
  }

  /** Velocity PID control (RPM) */
  public void setVelocity(double leftRPM, double rightRPM) {
    motorFrontLeft.setControl(velocityRequest.withVelocity(leftRPM));
    motorFrontRight.setControl(velocityRequest.withVelocity(rightRPM));

    currentMode = DriveMode.VELOCITY; // Update current mode
    desiredLeftRPM = leftRPM;
    desiredRightRPM = rightRPM;
  }

  /** Position PID control (rotations) */
  public void setPosition(double leftRotations, double rightRotations) {
    // Get current positions
    double currentLeftRot = motorFrontLeft.getPosition().getValueAsDouble();
    double currentRightRot = motorFrontRight.getPosition().getValueAsDouble();

    // Calculate velocity command using PID based on position error
    double leftVelocity = pid.calculate(currentLeftRot, leftRotations);
    double rightVelocity = pid.calculate(currentRightRot, rightRotations);

    // Apply calculated velocities
    motorFrontLeft.setControl(velocityRequest.withVelocity(leftVelocity));
    motorFrontRight.setControl(velocityRequest.withVelocity(rightVelocity));

    currentMode = DriveMode.POSITION; // Update current mode
  }

  /** Logging */
  private void logF() {
    // Log current control mode
    Logger.recordOutput("Drive/Mode", currentMode == null ? "NONE" : currentMode.name());

    // LEFT motor logs
    Logger.recordOutput("Drive/Left/Temp", motorFrontLeft.getDeviceTemp().getValue());
    Logger.recordOutput("Drive/Left/VelocityRPM", motorFrontLeft.getVelocity().getValue());
    Logger.recordOutput("Drive/Left/Position", motorFrontLeft.getPosition().getValue());
    Logger.recordOutput("Drive/Left/Voltage", motorFrontLeft.getMotorVoltage().getValue());
    Logger.recordOutput("Drive/Left/Current", motorFrontLeft.getStatorCurrent().getValue());
    Logger.recordOutput("Drive/Left/OutputPercent", motorFrontLeft.getDutyCycle().getValue());

    // RIGHT motor logs
    Logger.recordOutput("Drive/Right/Temp", motorFrontRight.getDeviceTemp().getValue());
    Logger.recordOutput("Drive/Right/VelocityRPM", motorFrontRight.getVelocity().getValue());
    Logger.recordOutput("Drive/Right/Position", motorFrontRight.getPosition().getValue());
    Logger.recordOutput("Drive/Right/Voltage", motorFrontRight.getMotorVoltage().getValue());
    Logger.recordOutput("Drive/Right/Current", motorFrontRight.getStatorCurrent().getValue());
    Logger.recordOutput("Drive/Right/OutputPercent", motorFrontRight.getDutyCycle().getValue());
    
    // Desired output logs
    Logger.recordOutput("Drive/Desired/LeftRPM", desiredLeftRPM);
    Logger.recordOutput("Drive/Desired/RightRPM", desiredRightRPM);
  }

  /** Update PID from NetworkTables */
  private void updatePID() {
    // Update PID from nettables
    double posKPcd = pidTable.getEntry("PosPID/posKPcd").getDouble(0.0);
    double posKIcd = pidTable.getEntry("PosPID/posKIcd").getDouble(0.0);
    double posKDcd = pidTable.getEntry("PosPID/posKDcd").getDouble(0.0);
    double velKPcd = pidTable.getEntry("VelPID/velKPcd").getDouble(0.0);
    double velKIcd = pidTable.getEntry("VelPID/velKIcd").getDouble(0.0);
    double velKDcd = pidTable.getEntry("VelPID/velKDcd").getDouble(0.0);

    pid.setPID(posKPcd, posKIcd, posKDcd);
    configLeft.Slot0.kP = velKPcd;
    configLeft.Slot0.kI = velKIcd;
    configLeft.Slot0.kD = velKDcd;
    configRight.Slot0.kP = velKPcd;
    configRight.Slot0.kI = velKIcd;
    configRight.Slot0.kD = velKDcd;
    motorFrontLeft.getConfigurator().apply(configLeft);
    motorFrontRight.getConfigurator().apply(configRight);
  }

  @Override
  public void periodic() {
    logF();
    updatePID();
  }
}
package frc.robot.subsystems;

import frc.robot.consts;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX motorFrontRight = new TalonFX(consts.CANID.RCanID);
  private final TalonFX motorFrontLeft = new TalonFX(consts.CANID.LCanID);
  private final PIDController pid = new PIDController(consts.PosPID.posKP.get(), consts.PosPID.posKI.get(), consts.PosPID.posKD.get());
  private final TalonFXConfiguration leftConfig = genConfig(true);
  private final TalonFXConfiguration rightConfig = genConfig(false);

  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

  /** Store cached PIDs */
  private double cachedVelKP = consts.VelPID.velKP.get();
  private double cachedVelKI = consts.VelPID.velKI.get();
  private double cachedVelKD = consts.VelPID.velKD.get();
  private double cachedVelKS = consts.VelPID.velKS.get();
  private double cachedPosKP = consts.PosPID.posKP.get();
  private double cachedPosKI = consts.PosPID.posKI.get();
  private double cachedPosKD = consts.PosPID.posKD.get();

  /** Enum to track which PID mode is active */
  private enum DriveMode {VELOCITY, POSITION}
  private DriveMode currentMode = null;

  /** Desired RPM for left and right motors */
  private double desiredLeftRPM = 0.0;
  private double desiredRightRPM = 0.0;

  /** Initialize motors */
  public DriveSubsystem() {
    motorFrontLeft.getConfigurator().apply(leftConfig);
    motorFrontRight.getConfigurator().apply(rightConfig);
  }

  /** Generate a config for a motor */
  public TalonFXConfiguration genConfig(boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (inverted) {
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = consts.VelPID.velKP.get();
    slot0.kI = consts.VelPID.velKI.get();
    slot0.kD = consts.VelPID.velKD.get();
    slot0.kS = consts.VelPID.velKS.get();

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
    // DifferentialDrive
    // DifferentialDriveKinematics
    motorFrontRight.setControl(velocityRequest.withVelocity(rightVelocity));

    currentMode = DriveMode.POSITION; // Update current mode
  }

  /** Kinematics mode */

  private void logCur() {
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

  private void updateVelPID() {
    leftConfig.Slot0.kP = consts.VelPID.velKP.get();
    leftConfig.Slot0.kI = consts.VelPID.velKI.get();
    leftConfig.Slot0.kD = consts.VelPID.velKD.get();
    leftConfig.Slot0.kS = consts.VelPID.velKS.get();
    rightConfig.Slot0.kP = consts.VelPID.velKP.get();
    rightConfig.Slot0.kI = consts.VelPID.velKI.get();
    rightConfig.Slot0.kD = consts.VelPID.velKD.get();
    rightConfig.Slot0.kS = consts.VelPID.velKS.get();
    motorFrontLeft.getConfigurator().apply(leftConfig);
    motorFrontRight.getConfigurator().apply(rightConfig);
  }

  private void updatePosPID() {
    pid.setPID(
      consts.PosPID.posKP.get(),
      consts.PosPID.posKI.get(),
      consts.PosPID.posKD.get()
    );
  }

  private void checkPIDUpdate() {
    if (consts.VelPID.velKP.get() != cachedVelKP || 
        consts.VelPID.velKI.get() != cachedVelKI || 
        consts.VelPID.velKD.get() != cachedVelKD ||
        consts.VelPID.velKS.get() != cachedVelKS) {
      updateVelPID();
      cachedVelKP = consts.VelPID.velKP.get();
      cachedVelKI = consts.VelPID.velKI.get();
      cachedVelKD = consts.VelPID.velKD.get();
      cachedVelKS = consts.VelPID.velKS.get();
    }
    if (consts.PosPID.posKP.get() != cachedPosKP ||
        consts.PosPID.posKI.get() != cachedPosKI || 
        consts.PosPID.posKD.get() != cachedPosKD) {
      updatePosPID();
      cachedPosKP = consts.PosPID.posKP.get();
      cachedPosKI = consts.PosPID.posKI.get();
      cachedPosKD = consts.PosPID.posKD.get();
    }
  }

  @Override
  public void periodic() {
    logCur();
    checkPIDUpdate();
    }
  }
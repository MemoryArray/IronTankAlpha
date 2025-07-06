package frc.robot;

import frc.robot.utils.TunableNumber;

public final class consts {
  // Tuning mode
  public static final boolean TUNING = true;
  
  // CAN IDs
  public static final class CANID {
    public static final int LCanID = 0;
    public static final int RCanID = 1;
    public static final int armCanID = 2;
  }

  // Velocity PID
  public static final class VelPID {
    public static final TunableNumber velKP = new TunableNumber("velKP", 0.002225);
    public static final TunableNumber velKI = new TunableNumber("velKI", 0.0);
    public static final TunableNumber velKD = new TunableNumber("velKD", 0.0);
    public static final TunableNumber velKS = new TunableNumber("velKS", 0.0);
  }

  // Position PID
  public static final class PosPID {
    public static final TunableNumber posKP = new TunableNumber("posKP", 1.5);
    public static final TunableNumber posKI = new TunableNumber("posKI", 0.0);
    public static final TunableNumber posKD = new TunableNumber("posKD", 0.05);
  }

  // Arm PID
  public static final class ArmPID {
    public static final TunableNumber armKP = new TunableNumber("armKP", 0.0001);
    public static final TunableNumber armKI = new TunableNumber("armKI", 0.0);
    public static final TunableNumber armKD = new TunableNumber("armKD", 0.0);
  }
  
  // Arm target angle
  public static final TunableNumber armTargetAngle = new TunableNumber("Arm Target Angle", 20.0);

  // Max values
  public static final class Limits {
    // This value controls the maximum of which the drive motors can run in
    // Real drive RPM = proportion * maxDriveRPMcd, proportion in [0, 1]
    public static final double maxDriveRPM = 300.0;
    public static final double armMaxCurrent = 40.0; // Max current for arm motor
    public static final double armStallCurrent = 30.0; // Max current for arm motor to trigger failsafe
    public static final double armStallThreshold = 1; // Threshold for arm stall detection, in RPM
  }
  
}

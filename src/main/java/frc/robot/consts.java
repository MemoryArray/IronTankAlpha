package frc.robot;

import lib.ntext.NTParameter;

/* const naming standards
 * nameName + c (const) + type
 */

public final class consts {
  // CAN IDs
  public static final class CANID {
    public static final int LCanIDci = 0;
    public static final int RCanIDci = 1;
    public static final int armCanIDci = 2;
  }

  @NTParameter(tableName = "PID")
  public final static class PID {
  // Velocity PID
    public static final class VelPID {
      public static final double velKPcd = 0.001;
      public static final double velKIcd = 0.0;
      public static final double velKDcd = 0.0;
    }
  
    // Position PID
    public static final class PosPID {
      public static final double posKPcd = 1.5;
      public static final double posKIcd = 0.0;
      public static final double posKDcd = 0.05;
    }
  }

  // Max values
  public static final class Maximums {
    // TODO: This is an example. Value is untested
    // This value controls the maximum of which the drive motors can run in
    // Real drive RPM = proportion * maxDriveRPMcd, proportion in [0, 1]
    public static final double maxDriveRPMcd = 100.0;
  }
  
}

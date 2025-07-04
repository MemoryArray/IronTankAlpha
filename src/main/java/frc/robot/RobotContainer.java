package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // Single Xbox controller for driving
  CommandXboxController mainController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Default drive command: single joystick arcade drive using velocity control
    Command velocityArcadeDrive =
      m_driveSubsystem.run(() -> {
        double forwardInput = deadBand(-mainController.getLeftY(), 0.1);
        double turnInput = deadBand(mainController.getRightX(), 0.1);

        // Convert joystick input [-1..1] to RPM [-MAX_DRIVE_RPM..MAX_DRIVE_RPM]
        double forwardRPM = forwardInput * consts.Maximums.maxDriveRPM;
        double turnRPM = turnInput * consts.Maximums.maxDriveRPM;

        // Calculate left and right RPM for arcade drive
        double leftRPM = forwardRPM + turnRPM;
        double rightRPM = forwardRPM - turnRPM;

        m_driveSubsystem.setVelocity(leftRPM, rightRPM);
      });

    m_driveSubsystem.setDefaultCommand(velocityArcadeDrive);

    // Optionally bind other commands here, e.g.:
    // mainController.a().toggleOnTrue(new SomeOtherCommand(m_driveSubsystem));

    // Arm control
    // Bind the "A" button to set the current position as zero (reset encoder)
    mainController.a().onTrue(new InstantCommand(() -> m_armSubsystem.resetArmPositionToZero()));

    // Bind the "X" button to move the arm to the target position (tunable)
    mainController.x().onTrue(new InstantCommand(() -> m_armSubsystem.setArmPosition(consts.armTargetAngle.get())));

    // Bind the "B" button to move the arm back to zero
    mainController.b().onTrue(new InstantCommand(() -> m_armSubsystem.setArmPosition(0)));
  }

  // Deadband helper to avoid drift
  public static double deadBand(double value, double tolerance) {
    if (value < tolerance && value > -tolerance) {
      return 0;
    }
    return value;
  }

  public Command getAutonomousCommand() {
    // Placeholder for autonomous command
    return new Command() {};
  }
}

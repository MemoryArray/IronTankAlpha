package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(); // Add ArmSubsystem instance

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
        double turnInput = deadBand(mainController.getLeftX(), 0.1);

        // Convert joystick input [-1..1] to RPM [-MAX_DRIVE_RPM..MAX_DRIVE_RPM]
        double forwardRPM = forwardInput * consts.Maximums.maxDriveRPMcd;
        double turnRPM = turnInput * consts.Maximums.maxDriveRPMcd;

        // Calculate left and right RPM for arcade drive
        double leftRPM = forwardRPM + turnRPM;
        double rightRPM = forwardRPM - turnRPM;

        m_driveSubsystem.setVelocity(leftRPM, rightRPM);
      });

    m_driveSubsystem.setDefaultCommand(velocityArcadeDrive);

    // Bind the "A" button to reset the arm position
    mainController.a().onTrue(new InstantCommand(() -> m_armSubsystem.resetArmPosition()));

    // Bind the "X" button to set the arm position to a desired angle (e.g., 45 degrees),
    // then move the arm back to the previously set armAngle
    mainController.x().onTrue(new InstantCommand(() -> {
        m_armSubsystem.setArmPosition(45.0); // Move to the desired angle
        m_armSubsystem.resetArmPositionToZero(); // Reset the motor position to 0
    }));
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

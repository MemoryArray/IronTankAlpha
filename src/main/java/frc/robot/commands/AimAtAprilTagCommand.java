package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.consts;

public class AimAtAprilTagCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final PIDController m_pidController;
    private static final String LIMELIGHT_NAME = "limelight";

    public AimAtAprilTagCommand(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

        m_pidController = new PIDController(0.01, 0.0, 0.0); // You may tune these
        m_pidController.setTolerance(3); // Degrees
        m_pidController.setSetpoint(0);  // Target is centered

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_pidController.reset();
    }

    @Override
    public void execute() {
        // If we don't see a target, stop
        if (LimelightHelpers.getFiducialID(LIMELIGHT_NAME) == -1) {
            m_driveSubsystem.setVelocity(0, 0);
            return;
        }

        double tx = LimelightHelpers.getTX(LIMELIGHT_NAME); // Horizontal offset in degrees
        double turningOutput = m_pidController.calculate(tx); // -1.0 to +1.0 output

        // Convert to turning RPM (symmetric, in-place turn)
        double turnRPM = turningOutput * consts.Limits.maxDriveRPM;

        double leftRPM = turnRPM;
        double rightRPM = -turnRPM;

        m_driveSubsystem.setVelocity(leftRPM, rightRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.setVelocity(0, 0);
    }

    @Override
    public boolean isFinished() {
        // You can uncomment if you want auto-stop when aimed
        // return m_pidController.atSetpoint();
        return false;
    }
}

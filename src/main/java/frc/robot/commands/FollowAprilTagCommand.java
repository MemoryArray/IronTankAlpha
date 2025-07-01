package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.consts;
import frc.robot.subsystems.DriveSubsystem;

public class FollowAprilTagCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final PIDController m_turnPid;
    private final PIDController m_distancePid;
    private static final String LIMELIGHT_NAME = "limelight";

    public FollowAprilTagCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;

        m_turnPid = new PIDController(0.01, 0, 0);
        m_distancePid = new PIDController(0.01, 0, 0);

        m_turnPid.setTolerance(3);
        m_distancePid.setTolerance(3);

        m_turnPid.setSetpoint(0);
        // TODO: Adjust this distance setpoint to your desired target Y value (height angle)
        m_distancePid.setSetpoint(0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_turnPid.reset();
        m_distancePid.reset();
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getFiducialID(LIMELIGHT_NAME) == -1) {
            m_driveSubsystem.setVelocity(0, 0);
            return;
        }

        double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

        double turningOutput = m_turnPid.calculate(tx);
        double forwardOutput = 0.0;

        if (m_turnPid.atSetpoint()) {
            forwardOutput = m_distancePid.calculate(ty);
        }

        // Convert outputs (assumed range roughly -1 to 1) to RPM
        double forwardRPM = forwardOutput * consts.Maximums.maxDriveRPMcd;
        double turnRPM = turningOutput * consts.Maximums.maxDriveRPMcd;

        // Calculate left and right RPMs for arcade drive
        double leftRPM = forwardRPM + turnRPM;
        double rightRPM = forwardRPM - turnRPM;

        m_driveSubsystem.setVelocity(leftRPM, rightRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.setVelocity(0, 0);
    }
}

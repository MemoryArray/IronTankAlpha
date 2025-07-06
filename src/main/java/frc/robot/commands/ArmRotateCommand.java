package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateCommand extends Command{
    private final ArmSubsystem m_ArmSubsystem;
    private final double rotations;

    public ArmRotateCommand(ArmSubsystem armSubsystem, double rotations) {
        this.m_ArmSubsystem = armSubsystem;
        this.rotations = rotations;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        m_ArmSubsystem.rotate(rotations);
    }
}

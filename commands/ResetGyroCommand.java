package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends CommandBase{
    
    private DriveSubsystem m_drive;

    public ResetGyroCommand(DriveSubsystem subsystem) {
        this.m_drive = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_drive.zeroHeading();
    }

}

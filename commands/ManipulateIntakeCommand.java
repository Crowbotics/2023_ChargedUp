package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeMode;

public class ManipulateIntakeCommand extends CommandBase{
    
    private IntakeSubsystem m_arm;
    private IntakeMode m_mode;

    public ManipulateIntakeCommand(IntakeSubsystem subsystem, IntakeMode mode) {
        this.m_arm = subsystem;
        this.m_mode = mode;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_arm.setIntakeMode(m_mode);
    }

}

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeMode;

public class AutoStopScoreCommand extends CommandBase{
    
    private IntakeSubsystem m_arm;

    public AutoStopScoreCommand(IntakeSubsystem subsystem) {
        this.m_arm = subsystem;
        m_arm.m_intakeEndMotor.getEncoder().setPosition(0);
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_arm.setIntakeMode(IntakeMode.OFF);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupt)
    {
        m_arm.m_intakeEndMotor.getEncoder().setPosition(0);
    }
}

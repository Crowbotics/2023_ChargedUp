package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeMode;

public class AutoScoreCommand extends CommandBase{
    
    private IntakeSubsystem m_arm;

    public AutoScoreCommand(IntakeSubsystem subsystem) {
        this.m_arm = subsystem;
        m_arm.m_intakeEndMotor.getEncoder().setPosition(0);
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_arm.setIntakeMode(IntakeMode.SCORE_CONE);
    }

    @Override
    public boolean isFinished()
    {
        if(Math.abs(m_arm.m_intakeEndMotor.getEncoder().getPosition()) > 10.0)
            return true;
        else
            return false;
    }
}

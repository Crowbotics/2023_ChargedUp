package frc.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMode;

public class AutoArmUpCommand extends CommandBase{
    
    private ArmSubsystem m_arm;

    public AutoArmUpCommand(ArmSubsystem subsystem) {
        this.m_arm = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_arm.moveArm(ArmMode.UP);
    }

    @Override
    public boolean isFinished()
    {
        if(Math.abs(m_arm.m_armMotor.getEncoder().getPosition() - m_arm.CRADLE) < 2.0)
            return true;
        else
            return false;
    }
}

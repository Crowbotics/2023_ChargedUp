package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMode;

public class AutoArmDownCommand extends CommandBase{
    
    private ArmSubsystem m_arm;

    public AutoArmDownCommand(ArmSubsystem subsystem) {
        this.m_arm = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_arm.moveArm(ArmMode.DOWN);
    }

    @Override
    public boolean isFinished()
    {
        
        if(Math.abs(m_arm.m_armMotor.getEncoder().getPosition() - m_arm.GRAVE) < 2.0)
            return false;
        else
            return true;

    }
}

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMode;

public class ManipulateArmCommand extends CommandBase{
    
    private ArmSubsystem m_arm;
    private ArmMode m_mode;

    public ManipulateArmCommand(ArmSubsystem subsystem, ArmMode mode) {
        this.m_arm = subsystem;
        this.m_mode = mode;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
       m_arm.moveArm(m_mode);
    }
}

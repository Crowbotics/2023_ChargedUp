package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase{

    public final CANSparkMax m_intakeEndMotor;
    private final CANSparkMax m_intakeMidMotor;


    public enum IntakeMode{
        OFF, COLLECT_CONE, COLLECT_CUBE, SCORE_CONE, SCORE_CUBE
    }

    public IntakeSubsystem()
    {
        m_intakeMidMotor = new CANSparkMax(ArmConstants.kIntakeMidCANID, MotorType.kBrushless);
        m_intakeEndMotor = new CANSparkMax(ArmConstants.kIntakeEndCANID, MotorType.kBrushless);
    }

    public void setIntakeMode(IntakeMode m)
    {
        switch(m){
        case OFF: 
            m_intakeMidMotor.set(0);
            m_intakeEndMotor.set(0);
            break;

        case COLLECT_CUBE:
            m_intakeMidMotor.set(ArmConstants.kIntakeCubeSpeed);
            m_intakeEndMotor.set(ArmConstants.kIntakeCubeSpeed);
            break;

        case COLLECT_CONE:
        m_intakeMidMotor.set(ArmConstants.kIntakeConeSpeed);
        m_intakeEndMotor.set(ArmConstants.kIntakeConeSpeed);
            break;

        case SCORE_CUBE:
        m_intakeMidMotor.set(ArmConstants.kScoreCubeSpeed);
        m_intakeEndMotor.set(ArmConstants.kScoreCubeSpeed);
            break;

        case SCORE_CONE:
        m_intakeMidMotor.set(ArmConstants.kScoreConeSpeed);
        m_intakeEndMotor.set(ArmConstants.kScoreConeSpeed);
            break;
        }
    }
    
}

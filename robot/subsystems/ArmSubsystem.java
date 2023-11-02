package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{

    public final double CRADLE = 0.0;
    public final double MID = 6.5;
    public final double GRAVE = 22.0;

    public final CANSparkMax m_armMotor;
    public double desiredPosition;
    public double val = 0;

    private SparkMaxPIDController armPID;

    public enum ArmMode{
        OFF, UP, MID, DOWN
    }

    public ArmMode m_armMode;

    public ArmSubsystem()
    {
        m_armMotor = new CANSparkMax(ArmConstants.kArmCANID, MotorType.kBrushless);
        m_armMotor.getEncoder().setPosition(0);
        m_armMode = ArmMode.OFF;
        
        m_armMotor.getEncoder().setPositionConversionFactor(1);

        armPID = m_armMotor.getPIDController();
        armPID.setFeedbackDevice(m_armMotor.getEncoder());
        armPID.setP(0.03);
        armPID.setD(0.001);
        armPID.setOutputRange(-0.5, 0.5);

        desiredPosition = m_armMotor.getEncoder().getPosition();
    }

    public void setArmMode(ArmMode choice)
    {
        m_armMode = choice;
    }

    public void manualMove()
    {

    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder:" , m_armMotor.getEncoder().getPosition());
        //SmartDashboard.putNumber("Arm Desired" , desiredPosition);
        //SmartDashboard.putNumber("Arm Calculated Value" , val);
        //SmartDashboard.putString("Arm String: ", m_armMode.toString());
    }

    public void moveArm(ArmMode mode) {

        setArmMode(mode);

        switch(m_armMode){
            case OFF: 
            ;
            break;

            case DOWN:  

            desiredPosition = Math.max(desiredPosition + 0.10, GRAVE);
            //desiredPosition = ArmConstants.kMoveArmSpeed;
            
            break;

            case MID:
            if(desiredPosition - MID < 0) // Too low
            {
                desiredPosition = Math.max(desiredPosition + 0.25, MID);
                desiredPosition = MID;
            }
            else if (desiredPosition - MID > 0) // Too high
            {
                desiredPosition = Math.max(desiredPosition - 0.25, MID);
                desiredPosition = MID;
            }
            
            break;

            case UP: 
            
            desiredPosition = Math.min(desiredPosition - .10, CRADLE);
            //desiredPosition = -ArmConstants.kMoveArmSpeed;
            break;        
        }

        desiredPosition = Math.max(desiredPosition, CRADLE);
        desiredPosition = Math.min(desiredPosition, GRAVE);

        armPID.setReference(desiredPosition, CANSparkMax.ControlType.kPosition);
        //m_armMotor.set(desiredPosition);
    }
}

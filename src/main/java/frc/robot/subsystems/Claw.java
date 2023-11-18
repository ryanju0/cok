package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Claw extends SubsystemBase {
    private CANSparkMax ClawMotor;
    private DoubleSolenoid clawSolenoid;
    private Timer clawRunningTimer = new Timer();

    public static final int kClawMotorCanId = 14;
    public static final double kNominalVoltage = 11.5;
    public static final int kClawMotorCurrentLimit = 12;

    public static final double kIntakeMotorSpeed = 1;
    public static final double kOuttakeMotorSpeed = -0.1;

    public Claw(){
        ClawMotor = new CANSparkMax(kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        ClawMotor.setSmartCurrentLimit(kClawMotorCurrentLimit);
        ClawMotor.setInverted(true);
        ClawMotor.enableVoltageCompensation(kNominalVoltage);
        ClawMotor.burnFlash();
    
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,7, 6);
    }
    public void clawIntakeCone(){
        ClawMotor.set(kNominalVoltage);
        clawSolenoid.set(Value.kReverse);
    }
    public void clawIntakeCube(){
        ClawMotor.set(kNominalVoltage);
        clawSolenoid.set(Value.kReverse);
    }
    public void dropPiece(){
        ClawMotor.set(0);
        clawSolenoid.set(Value.kReverse);
    }
    public boolean isCurrentSpikeDetected(){
        return (clawRunningTimer.get() > 0.5) && (ClawMotor.getOutputCurrent() > 20);
    }
    @Override
    public void periodic() {
            if (isCurrentSpikeDetected()){
                SmartDashboard.putBoolean("is piece in", isCurrentSpikeDetected());
                ClawMotor.set(0);
            }
      }
}

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax m_ShooterTop;
  private CANSparkMax m_ShooterBottom;
  private CANSparkMax m_Intake;
  private CANSparkMax m_Belly;
  //private final DoubleSolenoid p_arms;
  //private final Compressor c_compressor;
  public DigitalInput limitSwitch;

  public ShooterSubsystem() {
    m_ShooterTop = new CANSparkMax(60, MotorType.kBrushless);
    m_ShooterBottom = new CANSparkMax(61, MotorType.kBrushless);
    m_Intake = new CANSparkMax(15, MotorType.kBrushless);
    m_Belly = new CANSparkMax(17, MotorType.kBrushless);
    //p_arms  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    //c_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    limitSwitch = new DigitalInput(0);
  }

  // activates intake
  public void runIntake(double speed) {
        this.m_Intake.set(speed);
  }

  // runs belly
  public void runBelly(double speed) {
        this.m_Belly.set(speed);
  }

  // runs shooter
  public void runShooter(double speed) {
        m_ShooterTop.set(speed * 4);
        m_ShooterBottom.set(-speed);
  }

  //spins up shooter
  public void revUp(double speed, double ratio) {
    m_ShooterTop.set(speed * ratio); //4
    m_ShooterBottom.set(-speed); //1.5
  }

  // pushes note to shooter
  public void pushNote(DriveSubsystem hi) {
    hi.setX();
    double startTime = Timer.getFPGATimestamp();
    if(Timer.getFPGATimestamp() > startTime + .4) {
      m_Belly.set(1);
      m_Intake.set(1 / 12);
    } 
  }

  // runs intake and belly in unison to pick up note
  public void intakeNote(double speed) {
    m_Intake.set(speed);
    m_Belly.set(1);
  }


  // shoots a note
public void shoot(double s_Speed, double b_Speed, DriveSubsystem m_robotDrive, double ratio) {
    m_robotDrive.setX();
    double startTime = Timer.getFPGATimestamp();
    while(Timer.getFPGATimestamp() < startTime + 1.65) { 
        if (Timer.getFPGATimestamp() < startTime + .4) {
            m_Belly.set(-b_Speed / 5);
            m_Intake.set(-b_Speed / 20);
        }
        else if (Timer.getFPGATimestamp() > startTime + .4 && Timer.getFPGATimestamp() < startTime + 1.15) {
            m_ShooterTop.set(s_Speed * ratio); //4
            m_ShooterBottom.set(-s_Speed); //1.5
            m_Belly.set(0);
            m_Intake.set(0);
        }
        if(Timer.getFPGATimestamp() > startTime + 1.15 && Timer.getFPGATimestamp() < startTime + 1.55) {
            m_Belly.set(1);
            m_Intake.set(1);
        } 
        else if(Timer.getFPGATimestamp() > startTime + 1.6 && Timer.getFPGATimestamp() < startTime + 1.65) {
            stopAll();
        }
    }
}

public Command revIntakeCommand(double i_Speed) {
  return this.runOnce(() -> intakeNote(i_Speed));
}
public Command revIntakeCommand2(double i_Speed) {
  return this.runOnce(() -> runShooter(-i_Speed));
}

public Command notifyEndedCommand() {
  return this.runOnce(() -> System.out.println("Autonomous Ended"));
}

public Command intakeCommand(double i_Speed) {
    return this.runOnce(() -> intakeNote(i_Speed));
}

public Command stopShooterCommand(){
  return this.runOnce(() -> runShooter(-0.01));
}

public Command shooterCommand(double s_Speed, double b_Speed, DriveSubsystem m_robotDrive, double ratio) {
    return this.runOnce(() -> shoot(s_Speed, b_Speed, m_robotDrive, ratio));
}

  // stops all shooter and belly motors
  public void stopAll() {
        this.m_ShooterTop.set(-0.01);
        this.m_ShooterBottom.set(-0.01);
        this.m_Belly.set(-0.01);
        this.m_Intake.set(-0.01);
  }

  public Command stopAllCommand() {
      return this.runOnce(() -> this.stopAll());
  }


  /*  lifts up pneumatic shooter arms
  public void shooterUp() {
        p_arms.set(Value.kForward);
  }
  // lowers down pneumatic shooter arms
  public void shooterDown() {
        p_arms.set(Value.kReverse);
  }
  public boolean isShooterUp() {
    if (p_arms.get() == Value.kForward) {
      return true;
    }
    else {
      return false;
    }
  }
  
  public Command shooterDownCommand() {
    return this.runOnce(() -> this.shooterDown());
  }

  public Command shooterUpCommand() {
    return this.runOnce(() -> this.shooterUp());
  }

  public void compressorSwitch() {
      if(c_compressor.getPressureSwitchValue() == false){
      c_compressor.enableDigital();
    }
    else{
      c_compressor.disable();
    }
  }
*/




  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
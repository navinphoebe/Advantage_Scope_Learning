package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SecondMechanism extends SubsystemBase {
  /** Creates a new Arm. */
  public double angle1;
  public Mechanism2d mech = new Mechanism2d(4, 2);
  public MechanismLigament2d m_wrist;
  public MechanismRoot2d root;

  public SecondMechanism() {
    angle1 = 100;
    // the main mechanism object
    // the mechanism root node
    root = mech.getRoot("Part 1", 3, 0);
    m_wrist = root.append(new MechanismLigament2d("Part 2", 3, 50, 2, new Color8Bit(Color.kRed)));
    SmartDashboard.putData("Second Mechanism", mech);
  }

  public Command Position1() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          angle1 = 25;
        });
  }

  public Command Position2() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          angle1 = 200;
        });
  }

  @Override
  public void periodic() {
    m_wrist.setAngle(angle1);
    SmartDashboard.putData("Second Mechanism", mech);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public double angle1;
  public double angle2;
  public Mechanism2d mech = new Mechanism2d(100, 100);
  public MechanismLigament2d m_wrist;
  public MechanismLigament2d m_wrist2;
  public MechanismLigament2d m_wrist3;
  public MechanismLigament2d m_wrist4;
  public MechanismLigament2d m_wrist5;
  public MechanismRoot2d root;

  public Arm() {
    angle1 = -100;
    // angle2 = 100;
    // the main mechanism object
    // the mechanism root node
    root = mech.getRoot("climber", 75, 0);
    m_wrist = root.append(new MechanismLigament2d("base", 40, 180, 6, new Color8Bit(Color.kBlack)));
    m_wrist2 = m_wrist.append(new MechanismLigament2d("base1", 16, -90, 6, new Color8Bit(Color.kBlack)));
    m_wrist3 = m_wrist2.append(new MechanismLigament2d("wrist1", 40.0, -100, 6, new Color8Bit(Color.kRed)));
    m_wrist4 = m_wrist3.append(new MechanismLigament2d("wrist2", 8.0, 130, 6, new Color8Bit(Color.kGreen)));
    m_wrist5 = m_wrist3.append(new MechanismLigament2d("wrist3", 8.0, -50, 6, new Color8Bit(Color.kGreen)));
    SmartDashboard.putData("First Mechanism", mech);
  }
/* 
  public Command initialPosition() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          angle1 = 100;
          angle2 = 100;
        });
  }

  public Command groundPickup() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          angle1 = 170;
          angle2 = 70;
        });
  }

  public Command defendedScoring() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          angle1 = 160;
          angle2 = 60;
        });
  }

  public Command ampScoring() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          angle1 = 90;
          angle2 = -160;
        });
  }  */

  @Override
  public void periodic() {
    m_wrist3.setAngle(angle1);
    SmartDashboard.putData("First Mechanism", mech);
  }
}

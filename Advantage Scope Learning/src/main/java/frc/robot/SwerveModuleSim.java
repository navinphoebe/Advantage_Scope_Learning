// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SwerveModuleSim {
    public Timer timer = new Timer();

    public double meters;

    public SwerveModuleSim() {
        timer.start();
        meters = 0;
    }

    public double getValue(double speedMetersPerSecond) {
        double time = timer.get();
        timer.reset();
        double value = speedMetersPerSecond * time;
        meters += value;
        return meters;
    }
}

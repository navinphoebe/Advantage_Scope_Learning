// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Gyro {

    public Timer rotateTimer = new Timer();

    public double rotateValue;

    public Gyro() {
        rotateTimer.start();

        rotateValue = 0;
    }

    public double getGyroValueAdded(double omegaRadiansPerSecond) {
        double time = rotateTimer.get();
        rotateTimer.reset();
        double value = omegaRadiansPerSecond * time;
        rotateValue += value;
        return rotateValue;
    }
}
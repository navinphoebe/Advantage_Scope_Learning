// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Gyro {

    public Timer flTimer = new Timer();
    public Timer frTimer = new Timer();
    public Timer blTimer = new Timer();
    public Timer brTimer = new Timer();
    public Timer rotateTimer = new Timer();

    public double flValue;
    public double frValue;
    public double blValue;
    public double brValue;
    public double rotateValue;

    public Gyro() {
        flTimer.start();
        frTimer.start();
        blTimer.start();
        brTimer.start();
        rotateTimer.start();

        flValue = 0;
        frValue = 0;
        blValue = 0;
        brValue = 0;
        rotateValue = 0;
    }

    public double getFLValue(double speedMetersPerSecond) {
        double time = flTimer.get();
        flTimer.reset();
        double value = speedMetersPerSecond * time;
        flValue += value;
        return flValue;
    }

    public double getFRValue(double speedMetersPerSecond) {
        double time = frTimer.get();
        frTimer.reset();
        double value = speedMetersPerSecond * time;
        frValue += value;
        return frValue;
    }

    public double getBLValue(double speedMetersPerSecond) {
        double time = blTimer.get();
        blTimer.reset();
        double value = speedMetersPerSecond * time;
        blValue += value;
        return blValue;
    }

    public double getBRValue(double speedMetersPerSecond) {
        double time = brTimer.get();
        brTimer.reset();
        double value = speedMetersPerSecond * time;
        brValue += value;
        return brValue;
    }

    public double getGyroValueAdded(double omegaRadiansPerSecond) {
        double time = rotateTimer.get();
        rotateTimer.reset();
        double value = omegaRadiansPerSecond * time;
        rotateValue += value;
        return rotateValue;
    }
}

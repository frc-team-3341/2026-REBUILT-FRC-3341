
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface EasySwerveModuleIO {
    @AutoLog
    public static class EasySwerveModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(EasySwerveModuleIOInputs inputs) {
    }

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {
    }

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {
    }

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {
    }

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.vision.CyborgCommandVision;
import frc.robot.util.Util;

public class SubsystemReceiver extends SubsystemBase {
  DatagramSocket socket;
  private byte[] buffer;
  private double[] lastData;
  private double[] yawAverage = new double[3];
  private double[] flywheelAverage = new double[3];
  private int yawIndex = 0;
  private int flywheelIndex = 0;

  /** Creates a new SubsystemReciever. */
  public SubsystemReceiver() {
    setDefaultCommand(new CyborgCommandVision(this));
    try {
      socket = new DatagramSocket(7777);
    } catch (IOException e) {
    }

    this.buffer = new byte[256];
    this.lastData = new double[5];
    // isFound, yaw position, pitch postion, rpm, ground distance
    Thread listener = new Thread(() -> {
      while (!Thread.interrupted()) {
        try {
          DatagramPacket receivePacket = new DatagramPacket(buffer, 256);
          socket.receive(receivePacket);
          String[] newData = new String(receivePacket.getData()).trim().split(",");
          SmartDashboard.putString("Vision Data", String.join(",", newData));
          if (Double.parseDouble(newData[0]) == 1D && lastData[1] != Double.parseDouble(newData[1])) {
            for (int i = 0; i < newData.length; i++) {
              double value = Double.parseDouble(newData[i]);
              lastData[i] = value;
            }

            if (Math.abs(lastData[1]) > 0.025D) {
              double newYawTarget = RobotContainer.SUB_TURRET.yawEncoder.getPosition() + lastData[1] * 73.982D + 1D;
              if (newYawTarget < -5D) {
                yawAverage[yawIndex] = newYawTarget;
                yawIndex = (yawIndex + 1) % yawAverage.length;
              }
            }
            // lastData[3] += 100D;

            lastData[3] = Math.abs(lastData[3]) > 6000 ? 6000D : Math.abs(lastData[3]);
            flywheelAverage[flywheelIndex] = lastData[3];
            flywheelIndex = (flywheelIndex + 1) % flywheelAverage.length;
          }

        } catch (IOException e) { // thrown when the socket cannot receive the packet
          DriverStation.reportError("Cannot parse data", true);
        }
      }
    });

    listener.start();
  }

  public void adjustTurretFromData() {

  }

  @Override
  public void periodic() {
    if (lastData[0] == 1) {
      Preferences.setDouble("Yaw PID Target Position",
          (Util.summation(yawAverage)) / yawAverage.length);
      Preferences.setDouble("Pitch PID Target Position", lastData[2]);
      lastData[3] = lastData[3] > 6000 ? 6000D : lastData[3];
      lastData[3] = Math.abs(lastData[3]);
      Preferences.setDouble("FW Velocity Target", Util.summation(flywheelAverage) / flywheelAverage.length);
    }
    SmartDashboard.putNumber("Ground Distance to Target", Util.roundTo(lastData[4], 1));
  }
}

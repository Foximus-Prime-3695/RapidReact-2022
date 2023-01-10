// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * All usb cameras are run by this class.
 * 
 * @author Brach Knutson
 * @author Dylan Middendorf
 */
public class CameraHub {

    public CameraHub() {
        Thread pickupCamera = new Thread(() -> {
            CameraServer.startAutomaticCapture(0);
            CvSink sink = CameraServer.getVideo();
            CvSource src = CameraServer.putVideo("Front", 192, 144);
            Mat img = new Mat();
            while (!Thread.interrupted()) {
                if (sink.grabFrame(img) != 0) {
                    src.putFrame(img);
                } else {
                    DriverStation.reportWarning("COULD NOT READ IMAGE", false);
                }
            }
        });
        pickupCamera.start();
    }
}

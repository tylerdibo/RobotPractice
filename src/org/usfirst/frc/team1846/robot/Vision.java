package org.usfirst.frc.team1846.robot;

import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class Vision {
	USBCamera camUSB;
    CameraServer camServ;
    Image image;
    
    public Vision(String camLocation){
    	camUSB = new USBCamera(camLocation);
        camServ = CameraServer.getInstance();
        camServ.startAutomaticCapture(camUSB);
    }

    public void getColor(){
    	camUSB.getImage(image);
    	
    }
    
}

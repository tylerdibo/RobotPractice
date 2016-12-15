package org.usfirst.frc.team1846.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTest {
	//internal class to write to myRobot (a RobotDrive object) using a PIDOutput
	
	PIDController pidController;
	RobotDrive rDrive;
	double kp, ki, kd;
	
	public PIDTest(RobotDrive myDrive, Encoder enc, ADXRS450_Gyro gyro){
		kp = 0.1;
		ki = 0;
		kd = 0.6;
		pidController = new PIDController(kp, ki, kd, enc, new PIDDrive(gyro));
		rDrive = myDrive;
		LiveWindow.addActuator("Drive", "PID Encoder", pidController);
		LiveWindow.addSensor("Drive", "Encoder L", enc);
		pidController.setInputRange(-5000, 5000);
    	pidController.setOutputRange(-0.4, 0.4);
	}
	
    public class PIDDrive implements PIDOutput {
    	
    	ADXRS450_Gyro gyro;
    	
    	public PIDDrive(ADXRS450_Gyro gyro){
    		this.gyro = gyro;
    	}
    	
	    @Override
	    public void pidWrite(double output) {
	    	/*if(output > 0){
		    	rDrive.drive(-output, -gyro.getAngle()*0.02); //drive robot from PID output
	    	}else{
	    		rDrive.drive(-output, gyro.getAngle()*0.02); //drive robot from PID output
	    	}*/
	    	
	    	rDrive.drive(-output, 0); //drive robot from PID output

	    }
    }
    
    public void goToSetpoint(double setpoint){
    	pidController.setSetpoint(setpoint);
    	pidController.enable();
    }
    
    public void initializeTestMode(){
    	pidController.startLiveWindowMode();
    	pidController.enable();
    }
    
    public void disable(){
    	pidController.disable();
    }
    
    public void setPIDGain(double kp, double ki, double kd){
    	this.kp = kp;
    	this.ki = ki;
    	this.kd = kd;
    }
    
    public void updateDashboard(){
    	SmartDashboard.putData("PID Encoder", pidController);
    	SmartDashboard.putNumber("PID Encoder Output", pidController.get());
    }
}

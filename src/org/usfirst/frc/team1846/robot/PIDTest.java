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
	
	PIDController pidDrive, pidTurn;
	RobotDrive rDrive;
	double[] kDrive = {0.15, 0.05, 0.6}; //0.15, 0.05, 0.55
	double[] kTurn = {0.025, 0, 0.1}; //0.15, 0.05, 0.55
	
	public PIDTest(RobotDrive myDrive, Encoder enc, ADXRS450_Gyro gyro){
		//Init pid drive
		pidDrive = new PIDController(kDrive[0], kDrive[1], kDrive[2], enc, new PIDDrive(gyro));
		rDrive = myDrive;
		LiveWindow.addActuator("Drive", "PID Encoder", pidDrive);
		LiveWindow.addSensor("Drive", "Encoder L", enc);
		pidDrive.setInputRange(-5000, 5000);
    	pidDrive.setOutputRange(-0.4, 0.4);
    	
    	//Init pid turning
    	pidTurn = new PIDController(kTurn[0], kTurn[1], kTurn[2], gyro, new PIDTurn());
    	LiveWindow.addActuator("Drive", "PID Gyro", pidTurn);
		LiveWindow.addSensor("Drive", "Gyro", gyro);
    	pidTurn.setOutputRange(-0.4, 0.4);
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
    
    public class PIDTurn implements PIDOutput {	
	    @Override
	    public void pidWrite(double output) {
	    	/*if(output > 0){
		    	rDrive.drive(-output, -gyro.getAngle()*0.02); //drive robot from PID output
	    	}else{
	    		rDrive.drive(-output, gyro.getAngle()*0.02); //drive robot from PID output
	    	}*/
	    	
	    	rDrive.drive(-output, 1.0); //drive robot from PID output

	    }
    }
    
    public void goToSetpoint(double setpoint){
    	pidTurn.disable();
    	pidDrive.setSetpoint(setpoint);
    	pidDrive.enable();
    }
    
    public void turnToSetpoint(double setpoint){
    	pidDrive.disable();
    	pidTurn.setSetpoint(setpoint);
    	pidTurn.enable();
    }
    
    public void initializeTestMode(){
    	pidTurn.startLiveWindowMode();
    	pidTurn.enable();
    }
    
    public void disable(){
    	pidDrive.disable();
    	pidTurn.disable();
    }
    
    public void setPIDDrive(double kp, double ki, double kd){
    	kDrive[0] = kp;
    	kDrive[1] = ki;
    	kDrive[2] = kd;
    }
    
    public void updateDashboard(){
    	SmartDashboard.putData("PID Encoder", pidDrive);
    	SmartDashboard.putNumber("PID Encoder Output", pidDrive.get());
    	SmartDashboard.putData("PID Gyro", pidTurn);
    	SmartDashboard.putNumber("PID Gyro Output", pidTurn.get());
    }
}

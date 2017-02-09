
package org.usfirst.frc.team1846.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    
    double robotX, robotY, robotAngle;
    
    VictorSP leftF, leftR, rightF, rightR;
    RobotDrive drive;
    Joystick stick;
    ADXRS450_Gyro gyro;
    AnalogInput analogEncL, analogEncR;
    AnalogTrigger analogTrigL, analogTrigR;
    Counter encLCount, encRCount;
    Encoder encL, encR, grayEnc;
    BuiltInAccelerometer accel;
    //Servo camX, camY;
    
    Vision vis;
    PIDTest pid;
    UDP udp;
    
    SerialPort serial;
    I2C i2c;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {    	
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
        robotX = robotY = 0;
        robotAngle = 0;
        
        /*camX = new Servo(8);
        camY = new Servo(9);
        vis = new Vision("cam2");*/
        
        leftF = new VictorSP(2);
        leftR = new VictorSP(0);
        rightF = new VictorSP(3);
        rightR = new VictorSP(1);
        drive = new RobotDrive(leftR, leftF, rightR, rightF);
        drive.setExpiration(0.1);
        
        stick = new Joystick(0);
        
        analogEncL = new AnalogInput(0);
        analogEncR = new AnalogInput(1);
        analogTrigL = new AnalogTrigger(analogEncL);
        analogTrigL.setFiltered(true);
        encLCount = new Counter(analogTrigL);
        //analogTrigL.setLimitsRaw(lower, upper);
        analogTrigR = new AnalogTrigger(analogEncR);
        analogTrigR.setFiltered(true);
        encRCount = new Counter(analogTrigR);
        encL = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
        encL.setDistancePerPulse(1/28.9);
        encR = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
        encR.setDistancePerPulse(1/20.5);
        grayEnc = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        
        pid = new PIDTest(drive, encL, gyro);
        try {
			udp = new UDP();
			udp.start();
		} catch (IOException e) {
			//udp initialization failed
			e.printStackTrace();
		}
        
        //serial = new SerialPort(115200, Port.kUSB);
        i2c = new I2C(I2C.Port.kOnboard, 0x11);
        
        accel = new BuiltInAccelerometer();
        
        LiveWindow.addActuator("Drive", "Left Front", leftF);
        LiveWindow.addActuator("Drive", "Left Rear", leftR);
        LiveWindow.addActuator("Drive", "Right Front", rightF);
        LiveWindow.addActuator("Drive", "Right Rear", rightR);
        LiveWindow.addSensor("Localization", "Gyro", gyro);
        
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	gyro.reset();
    	encL.reset();
    	encR.reset();
    	//pid.goToSetpoint(60);
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	updateDashboard();
    	drive.drive(-0.2, -gyro.getAngle()*0.02); //Line follower
    	//updatePosition();
    	//driveToTarget(50, 50);
    	
    	Timer.delay(0.005);
    }
    
    public void teleopInit(){
    	gyro.reset();
    	pid.disable();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        drive.arcadeDrive(stick);
        updateDashboard();
        updatePosition();
        /*byte[] bytes = {(byte) 255};
        serial.write(bytes, 1);*/
        
        if(stick.getTrigger()){
        	driveInches(48);
        	turnToDegrees(90);
        	driveInches(48);
        	turnToDegrees(180);
        	driveInches(48);
        	turnToDegrees(270);
        	driveInches(48);
        	turnToDegrees(360);
        }
        /*if(stick.getTrigger()){
        	while(((encL.getDistance() + encR.getDistance()) / 2) < 220){
            	drive.drive(-0.3, -gyro.getAngle()*0.02);
        	}
        	while(gyro.getAngle() > -90){
            	drive.drive(-0.2, -1.0);
        	}
        	encL.reset();
        	encR.reset();
        	while(((encL.getDistance() + encR.getDistance()) / 2) < 60){
            	drive.drive(-0.3, -(gyro.getAngle()+90)*0.02);
        	}
        	while(gyro.getAngle() < 90){
            	drive.drive(-0.2, 1.0);
        	}
        	encL.reset();
        	encR.reset();
        	while(((encL.getDistance() + encR.getDistance()) / 2) < 60){
            	drive.drive(-0.3, -(gyro.getAngle()-90)*0.02);
        	}
        	while(gyro.getAngle() > 180){
            	drive.drive(-0.2, -1.0);
        	}
        	encL.reset();
        	encR.reset();
        	while(((encL.getDistance() + encR.getDistance()) / 2) < 220){
            	drive.drive(-0.3, -gyro.getAngle()*0.02);
        	}
        }*/
        Timer.delay(0.005);
    }
    
    public void testInit(){
    	gyro.reset();
    	encL.reset();
    	encR.reset();
    	pid.initializeTestMode();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	gyro.updateTable();
    	updateDashboard();
    }
    
    private void updatePosition(){
    	double distance = (encL.getRaw() + encR.getRaw()) / 2;
    	encL.reset();
    	encR.reset();
    	robotAngle = Math.toRadians(gyro.getAngle());
    	robotX += Math.sin(robotAngle) * distance;
    	robotY += Math.cos(robotAngle) * distance;
    	SmartDashboard.putNumber("Robot X", robotX);
    	SmartDashboard.putNumber("Robot Y", robotY);
    }
    
    private void driveToTarget(double targetX, double targetY){
    	//Turn to target
    	double targetAngle = Math.atan((targetX-robotX) / (targetY-robotY));
    	SmartDashboard.putNumber("Target Angle", targetAngle);
    	while(Math.abs(targetAngle-robotAngle) > 0.1){
	    	if(targetAngle-robotAngle >= 0){
	    		drive.drive(0.3, -1.0);
	    	}else{
	    		drive.drive(0.3, 1.0);
	    	}
	    	updatePosition();
	    	updateDashboard();
	    	Timer.delay(0.002);
    	}
    	drive.drive(0, 0);
    }
    
    private void driveInches(double inches){
    	encL.reset();
    	pid.goToSetpoint(inches);
    	Timer.delay(0.2);
    	while(encR.getRate() != 0 && encL.getRate() != 0){
    		Timer.delay(0.1);
    	}
    	pid.disable();
    }
    
    private void turnToDegrees(double angle){
    	pid.turnToSetpoint(angle); //*1.04
    	Timer.delay(0.2);
    	while(encR.getRate() != 0 && encL.getRate() != 0){
    		Timer.delay(0.1);
    	}
    	pid.disable();
    }
    
    private void updateDashboard(){
    	pid.updateDashboard();
    	/*byte[] serialNums = serial.read(serial.getBytesReceived());
    	if(serialNums.length > 0){
    		SmartDashboard.putNumber("USB", serialNums[serialNums.length-1]);
    	}*/
    	byte[] i2cData = new byte[1];
    	i2c.read(0xBB, 1, i2cData);
    	SmartDashboard.putNumber("I2C Data", i2cData[0]);
    	SmartDashboard.putNumber("Speed L", encL.getRate());
    	SmartDashboard.putNumber("Speed R", encR.getRate());
    	SmartDashboard.putData("Left Front", leftF);
        SmartDashboard.putData("Left Rear", leftR);
        SmartDashboard.putData("Right Front", rightF);
        SmartDashboard.putData("Right Rear", rightR);
        SmartDashboard.putData("Gyro" , gyro);
        SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
        SmartDashboard.putData("Encoder Left", encL);
        SmartDashboard.putData("Encoder Right", encR);
        SmartDashboard.putNumber("Left enc", encL.getRaw());
        SmartDashboard.putNumber("Right enc", encR.getRaw());
        SmartDashboard.putData("Grayhill Encoder", grayEnc);
        SmartDashboard.putData("Analog Encoder Left", encLCount);
        SmartDashboard.putData("Analog Encoder Right", encRCount);
        SmartDashboard.putData("Accelerometer", accel);
        /*SmartDashboard.putData("Camera X", camX);
        SmartDashboard.putData("Camera Y", camY);*/
    }
    
}

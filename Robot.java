package org.usfirst.frc.team6236.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;


public class Robot extends SampleRobot {
	//Joysticks
	private Joystick leftStick;
	private Joystick rightStick;
    //Motor controllers
    private VictorSP left;
    private VictorSP right;
    private VictorSP climber;
	//Sensors
    private AnalogGyro gyro;
    //Signal light, uses PCM for a quick and easy support
    private Solenoid signalLight;
	//Software objects
    private double threshold = 0.15;
    private SendableChooser<String> autoSelect;
	private Timer methodTimer;
    
	//Initialization of hardware components
    public Robot() {
        leftStick = new Joystick(0);
        rightStick = new Joystick(1);
        left = new VictorSP(0);
        right = new VictorSP(1);
        climber = new VictorSP(2);
        signalLight = new Solenoid(0);
        gyro = new AnalogGyro(0);
    }
    
    
    
    public void robotInit() {
    	//Autonomous selector definitions 
    	autoSelect = new SendableChooser<String>();
    	autoSelect.addDefault("Default", "Default");
    	autoSelect.addObject("Left", "Left");
    	autoSelect.addObject("Center", "Center");
    	autoSelect.addObject("Right", "Right");
    	autoSelect.addObject("Test", "Test");
    	SmartDashboard.putData("Auto Select", autoSelect);
    	//Gyroscope setup
    	gyro.initGyro();
        gyro.calibrate();//Reset Gyro
        //Front camera
       	UsbCamera front = CameraServer.getInstance().startAutomaticCapture();
		front.setResolution(320, 240);
		//Rear camera
		UsbCamera back = CameraServer.getInstance().startAutomaticCapture();
		back.setResolution(320, 240);
    }
    
    public void autonomous(){
    	//Clear Gyro
    	gyro.reset();
    	
    	//Get selected string
    	String auto = (String) autoSelect.getSelected();
    	
		switch(auto){
			case "Left"://When on the left side of airship
				PIDStraight(1.5);
				GyroTurn(-45, 1);
				PIDStraight(1.5);
			break;
			case "Center"://When in the center spot
				PIDStraight(1.7);
			break;
			case "Right"://When on the right side of airship
				PIDStraight(1.5);
				GyroTurn(45, 1);
				PIDStraight(1.5);
			break;
			case "Test"://For testing
				//Testing code goes here
			break;
			case "Default":
			default:
				signalLight.set(true);
				Timer.delay(1);
				signalLight.set(false);
				Timer.delay(1);
				signalLight.set(true);
				Timer.delay(1);
				signalLight.set(false);
				Timer.delay(1);
				signalLight.set(true);
				Timer.delay(1);
				signalLight.set(false);
				Timer.delay(1);
			break;
		}
    }

    public void operatorControl() {
        //Clear Gyro
    	gyro.reset();
    	
    	while (isOperatorControl() && isEnabled()) {	
        	double leftY = -leftStick.getY();//Left is reversed
        	double rightY = rightStick.getY();//Right is normal
        	
        	if(Math.abs(leftY) > threshold){//Left speed with dead zone
        		left.set(leftY);
        	}
        	else{
        		left.set(0.0);//Left stop
        	}
        	
        	if(Math.abs(rightY) > threshold){//Right speed with dead zone
        		right.set(rightY);
        	}
        	else{
        		right.set(0.0);//Right stop
        	}
        	
        	        			
        	if(leftStick.getRawButton(3) || rightStick.getRawButton(3)){//Climber slow forward
        		climber.set(-0.5);
        	}
        	else if(leftStick.getRawButton(5) || rightStick.getRawButton(5)){//Climber slow backwards
        		climber.set(-1.0);
        	}
        	else{
        		climber.set(0.0);//Climber stop
        	}
        	
        	if(leftStick.getRawButton(4) || rightStick.getRawButton(4)){//Climber fast forward
        		climber.set(0.5);
        	}
        	else if(leftStick.getRawButton(6) || rightStick.getRawButton(6)){//Climber fast backwards
        		climber.set(1.0);
        	}
        	else{
        		climber.set(0.0);//Climber stop
        	}
        	
        	if(leftStick.getTrigger() || rightStick.getTrigger()){//Signal light controlled
        		signalLight.set(true);//Signal light on
        	}
        	else{
        		signalLight.set(false);//Signal light off
        	}
        	        	
        	SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        	SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
        }
    }
    
    public void setSpeed(double speed_L, double speed_R){//two input speed control
	   	left.set(-speed_L);
	   	right.set(speed_R);
   	}
   
   	public void setSpeed(double speed){//one input speed control
	   	setSpeed(speed, speed);
   	}
   
   	public void PIDStraight(double timeSecs){
	   	methodTimer = new Timer();//Timer for keeping track of time
	   	methodTimer.start();//Start timer
	   	
	   	MiniPID PID = new MiniPID(0.1, 0, 100);//Adding any integral at all causes heavy oscillation
   		
	   	double goal = gyro.getAngle();//Set goal to current angle
	   	
	   	while(methodTimer.get() < timeSecs){//while timer is less than time to move
   			double error = PID.getOutput(gyro.getAngle(), goal);//Error from PID
   			
   			DriverStation.reportError("Straight Loop is running", false);
   			
   			if(error < 0){//When error is negative
   				setSpeed(-0.4, -0.5);
   			}
   			else if(error > 0){//When error is positive
   				setSpeed(-0.5, -0.4);
   			}
   			else if(error == 0){//When error is 0
   				setSpeed(0);
  				DriverStation.reportError("Straight PID error is 0", false);//This is here for debugging purposes
   			}
   			else{//When things are broken
  				DriverStation.reportError("Straight PID error is somehow NaN", false);//Yeah if this happens things are broken
  			}
   		
   			SmartDashboard.putNumber("PID Target", goal);//Graphs
   			SmartDashboard.putNumber("PID Value", gyro.getAngle());//Graphs
   			SmartDashboard.putNumber("PID error", error);//Graphs
	   	}
	   	setSpeed(0);//Stop
	   	methodTimer.stop();//Stop timer
   	}
   	
   	public void GyroTurn(double angle, double timeout){//Right is negative
   		methodTimer = new Timer();//Timer for keeping track of time
	   	methodTimer.start();//Start timer
   		
	   	double goal = gyro.getAngle() + angle;//Target equals current angle + desired turn angle
	   	
	   	while(gyro.getAngle() != goal && methodTimer.get() < timeout){
	   		if(gyro.getAngle() < goal){//If negative the turn left
	   			setSpeed(-0.3, 0.3);
	   		}
	   		else if(gyro.getAngle() > goal){//If negative the turn right
	   			setSpeed(0.3, -0.3);
	   		}
	   		else{
	   			setSpeed(0);//Stop
	   		}
	   	}
	   	
	   	setSpeed(0);//Stop
	   	methodTimer.stop();//Stop timer
   	}
}
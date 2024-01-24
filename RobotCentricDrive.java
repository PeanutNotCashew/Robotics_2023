//this controls omnidirectional movement and one motor-powered arm
//bot uses mecanum wheels
//left joystick controls movement and strafing, shoulders control rotation
//right joystick controls arm

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp

public class RobotCentricDrive extends LinearOpMode {

    //wheel motors
    private DcMotor motor_fl;
    private DcMotor motor_fr;
    private DcMotor motor_bl;
    private DcMotor motor_br;
    
    //arm motor
    private DcMotor motor_arm;
    
    //servos
    private Servo wrist_servo;
    private CRServo claw;
    private Servo launch;
    
    
    @Override
    public void runOpMode() {
        
        
        
        //mapping variables to motors in control hub
        motor_fl = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_fl");
        motor_fr = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_fr");
        motor_bl = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_bl");
        motor_br = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_br");
        
        motor_arm = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_arm");
        motor_arm.setDirection(DcMotor.Direction.REVERSE);
        
        claw = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "claw_rot");
        wrist_servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "wrist_rot");
        launch = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "launcher");
        
        motor_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        motor_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        motor_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //setting up the IMU(gyroscope)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //orientation parameters
        IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu.initialize(imuParameters);
        
        motor_arm.setPower(0);
        
        //declaring variables
        double wristpos = 0;
        launch.setPosition(0.5);
        
        wrist_servo.scaleRange(0.03d, 0.97d);
        
        
        
        
        waitForStart();
        
        while (opModeIsActive()) {
            double factor = 1;
            double armFactor = 0.6;
            if (gamepad1.x) {factor = 0.5;}
            
            
            telemetry.addData("Status", "Operating as normal.");
            
            //this code sets values to different sticks
            double y = -gamepad1.left_stick_y; 
            double x = gamepad1.left_stick_x * 1.1;
            double r = this.gamepad1.right_trigger - this.gamepad1.left_trigger;
            

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //the values here make the motors turn and stuff
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
            double frontLeftPower = (y + x + r) / denominator;
            double backLeftPower = (y - x + r) / denominator;
            double frontRightPower = (y - x - r) / denominator;
            double backRightPower = (y + x - r) / denominator;
            
            //this sets the powers of the motors to the stuff we did above
            motor_fl.setPower(-frontLeftPower * factor);
            motor_bl.setPower(-backLeftPower * factor);
            motor_fr.setPower(frontRightPower * factor);
            motor_br.setPower(backRightPower * factor);
            
            //arm code
            double armPower = gamepad1.right_stick_y;
            motor_arm.setPower(armPower);
            telemetry.addData("arm position", motor_arm.getCurrentPosition());
            telemetry.addData("Pos", wrist_servo.getPosition());
            telemetry.addData("mloc", motor_fr.getCurrentPosition());
            telemetry.update();
            
            //claw code
            double clawPower = 0;
            if(gamepad1.right_bumper){
                clawPower = 1;
            }
            
            if(gamepad1.left_bumper){
                clawPower = -1;
            }
            
            claw.setPower(clawPower);
            
            //wrist code 
            double lastUpdateTime = getRuntime(); // Initialize the last update time
            if(wristpos > 1) {
                wristpos = 1;
            }
            if(wristpos < 0) {
                wristpos = 0;
            }
            while(gamepad1.dpad_right) {
                double currentTime = getRuntime();
                double elapsedTimeSinceUpdate = currentTime - lastUpdateTime;
                if (elapsedTimeSinceUpdate >= 0.01) { 
                    //wrist_servo.setDirection(Servo.Direction.REVERSE);
                    wristpos = wristpos + 0.005;
                    wrist_servo.setPosition(wristpos);
                    lastUpdateTime = getRuntime(); // Reset the last update time
                }
            }
            while(gamepad1.dpad_left) {
                double currentTime = getRuntime();
                double elapsedTimeSinceUpdate = currentTime - lastUpdateTime;
                if (elapsedTimeSinceUpdate >= 0.01) {
                    //wrist_servo.setDirection(Servo.Direction.FORWARD);
                    wristpos = wristpos - 0.005;
                    wrist_servo.setPosition(wristpos);
                    lastUpdateTime = getRuntime(); // Reset the last update time
                }
            }
            
            
            //launcher
            if(gamepad1.y) {
                launch.setPosition(1);
                //armFactor = 1;
            }

    }
}
    
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Camera;

import java.util.Timer;

@Autonomous
public class AutoMain extends LinearOpMode {
    private DcMotorEx leftTopMotor, rightTopMotor, leftBottomMotor, rightBottomMotor, arm, transfer, intake, shooter;
    private Servo claw, flicker, holder;
    private DcMotorEx[] motors;
    private final double TPI = 33.5625;
    private ElapsedTime shooterTime;
    private int distance;

    @Override
    public void runOpMode() throws InterruptedException  { //Load Zone B
        initialize();
        //0 Void 1 Forward 2 Reverse
        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwwardd
        //turnBot(2, 2, 1, 1, -24, .60); //Backward
        //turnBot(2, 1, 2, 1, 30, .60); //Straf right
        //turnBot(2, 1, 1, 2, 30, .60); //Rotating on the axis
        //Problem.. Find straf left.. Aman will check out the orevious line and try different combinations for straf left

        //turnBot(1, 2, 1, 2,30,.60 )//Rotate
        yeetRing();


    }

    public void initialize() {
        //Even if I'm not using it, I have to map it because it is mapped on the bot.
        leftTopMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBottomMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightTopMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBottomMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");
        holder = hardwareMap.servo.get("holder");
        motors = new DcMotorEx[]{leftTopMotor, rightTopMotor, leftBottomMotor, rightBottomMotor};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motor == leftTopMotor || motor == leftBottomMotor)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
        waitForStart();
    }

    /**
     * @param power
     * @param distance inchies so you will have to convert to tics
     */

    //This method accepts 6 variables:
    // 4 of them are motors (leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor)
    //These variables will accept 3 integers: 0 Void, 1 Forward, 2 Reverse
    //It also accepts power and distance

    //withIntake will be used later (work in progress)
    public void moveBot(int leftT, int leftB, int rightT, int rightB, int distance, double power, boolean withIntake) throws InterruptedException{
        if (leftT == 1) {
            leftTopMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(leftT == 2){
            leftTopMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftB == 1) {
            leftBottomMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(leftB == 2){
            leftBottomMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightT == 1) {
            rightTopMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(rightT == 2){
            rightTopMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightB == 1) {
            rightBottomMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(rightT == 2){
            rightBottomMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        /*if(withIntake) {
            while (shooterTime.milliseconds() <= 5000) {
                intake.setPower(.5);
                transfer.setPower(1);
                heartbeat();
            }
        }*/

        //Moves the robot
        int travel = (int) (distance * TPI);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(travel);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //This is what checks if the motors are supposed to be still running.
        while (leftTopMotor.isBusy() && rightTopMotor.isBusy() && leftBottomMotor.isBusy() && rightBottomMotor.isBusy()) {
            heartbeat();
        }
        //intake.setPower(0);
       // transfer.setPower(0);
    }

    public void yeetRing() throws InterruptedException { //NEEDS TO BE REVAMPED TO INCLUDE FLICKER AND PARAMETER FOR RINGS
        shooter.setPower(75);
        shooterTime = new ElapsedTime();
        while (shooterTime.milliseconds() <= 5000) {
            intake.setPower(-.5);
            transfer.setPower(1);
            flicker.setPosition(1);
            heartbeat();
        }
        flicker.setPosition(0);
        intake.setPower(0);
        transfer.setPower(0);
        //flicker.setPosition(0);
        shooter.setPower(0);
    }



    public void heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }

}

package org.firstinspires.ftc.teamcode;
/* Created Atul Errabolu and Kush on 7/25/2019 */
/* modified by Sebastian Ochoa on 1/3/21 */
//Modified by Aman Modi on 1/4/21
//restructured by Seb on 1/5/21
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleOP extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, arm, shooter, intake, transfer;
    private boolean direction, togglePrecision;
    private double factor;
    boolean currentB = false;
    boolean previousB = false;
    boolean currentRB = false;
    boolean previousRB = false;
    boolean clawState = false;
    boolean flicktime = false;
    double armPos = 0;
    private Servo claw, flicker, holder; //claw, flicker, holder
    boolean reverse;
    int reverseFactor;
    private BNO055IMU imu;
    private ElapsedTime runtime;
    private double servo;
    double shooterPower = .80;
    boolean shotMode = false;
    boolean clawReady = false;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime clawmove = new ElapsedTime();
    ElapsedTime flick = new ElapsedTime();
    boolean servoMoving = false;
    boolean armmove = false;
    final static double dropWobbleTime = 1000;


    //double initialAngle = currentAngle();
    @Override
    public void init() {
        //Maps all the variables to its respective hardware
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");
        holder = hardwareMap.servo.get("holder");
        //Initialize all the hardware to use Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initializing all new motors (shooter, arm, intake, transfer)
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setting the motors' power to 70

        //Initialize the motors to begin stationary
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Left Motors are in reverse and Right Motors are forward so the robot can move forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        runtime = new ElapsedTime();
        reverse = false;
    }


    @Override
    public void loop() {
        //if(!leftFront.isBusy()&&!leftBack.isBusy()&&!rightFront.isBusy()&&!rightBack.isBusy()) {
        //Increasing the power gradually
        //int power = (DcMotorSimple) arm.getPower();


        //toggles precision mode if the right stick button is pressed


        //sets the factor multiplied to the power of the motors
        factor = togglePrecision ? .3 : 1; //the power is 1/5th of its normal value while in precision mode

        // Do not mess with this, if it works, it works
        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
        double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
        double rightX = gamepad1.right_stick_x; // right stick x axis controls turning
        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle) + rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x * Math.cos(powerAngle) - rightX, -1.0, 1.0);


        //Set the position of arm to counter clockwise/clockwise


        //neutral is .5, right trigger .5 to 1, left trigger is 0 to .5 What???


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);

        //Incrementing the power by 0.0 EVERY TIME you call this function
        //Incrementing the power by 0.0 EVERY TIME you call this function
        // for jon in jon on jon

        //Updating the power of the motors
     /* arm.setPower(power);
     shooter.setPower(power);
     intake.setPower(power);
     transfer.setPower(power); */

        //Reset the intake and transfer encoders
        precisionMode(); //check for precision mode
        armTravel(); // move arm
        powerShot(); // toggles speed mode for flywheel
        revShoot(); // controls flywheel
        toggleIntake(); // controls intake, on off backwards
        flickRing(); // toggles flicker
        toggleHolder(); // toggles intake clip

        //}
    }

    public void revShoot() { // controls the flywheel WORKS
        if (gamepad1.right_trigger > .499999) {
            shooter.setPower(-shooterPower);
        } else {
            shooter.setPower(0);
        }
    }

    public void powerShot() { // lowers flywheel speed
        if (gamepad1.dpad_left && !shotMode) {
            shooterPower = .78;
            shotMode = true;
        } else if (gamepad1.dpad_right && shotMode) {
            shooterPower = .61;
            shotMode = false;
        }
    }

    public void toggleIntake() { // controls intake and transfer
        if (gamepad1.left_trigger > .499999) {
            intake.setPower(-1);
            transfer.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(1);
            transfer.setPower(-1);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    public void flickRing() { // controls flicker
 /* if (gamepad1.a && !servoMoving) {
 timer.reset();
 flicker.setPosition(1);
 servoMoving = true;
 }
 if (timer.milliseconds() >= 850 && servoMoving) {
 timer.reset();
 flicker.setPosition(0);
 servoMoving = false;
 }
*/
        if (gamepad1.a) {
            flicker.setPosition(.35);
        } else {
            flicker.setPosition(.64);
        } // use this code if the above code refuses to work.

     /*   if (gamepad1.a){
           flick.reset();

           flicktime = true;
        }
        if(flicktime){
            if(flick.milliseconds() > 0 && flick.milliseconds() < 250){
                flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 300 && flick.milliseconds() < 550){
                flicker.setPosition(.64);
            }
            if(flick.milliseconds() > 600 && flick.milliseconds() < 850){
                flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 900 && flick.milliseconds() < 1150){
                flicker.setPosition(.64);
            }
            if(flick.milliseconds() > 1200 && flick.milliseconds() < 1450){
                flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 1500){
                flicker.setPosition(.64);
                flicktime = false;
            }


        }
*/
    }

    public void precisionMode() { // controls precision mode
        if (gamepad2.left_stick_button || gamepad1.left_stick_button) {
            togglePrecision = true;
        } else if (gamepad2.right_stick_button || gamepad1.right_stick_button) {
            togglePrecision = false;
        }
    }

// public void snapBot(){
// if(gamepad1.back){
// while(Math.abs(currentAngle()-initialAngle)>=5){
// if(currentAngle()-initialAngle>0){
// leftFront.setDirection(DcMotor.Direction.FORWARD);
// leftBack.setDirection(DcMotor.Direction.FORWARD);
// rightFront.setDirection(DcMotor.Direction.FORWARD);
// rightBack.setDirection(DcMotor.Direction.FORWARD);
// leftFront.setPower(.1);
// leftBack.setPower(.1);
// rightFront.setPower(.1);
// rightBack.setPower(.1);
// }
// else{
// leftFront.setDirection(DcMotor.Direction.REVERSE);
// leftBack.setDirection(DcMotor.Direction.REVERSE);
// rightFront.setDirection(DcMotor.Direction.REVERSE);
// rightBack.setDirection(DcMotor.Direction.REVERSE);
// leftFront.setPower(.1);
// leftBack.setPower(.1);
// rightFront.setPower(.1);
// rightBack.setPower(.1);
// }
// }
// }
// }


    public void toggleHolder() {// not really useful, just to make it possible to toggle the holder if needed. KINDA WORKS
        if (gamepad1.dpad_up) {
            holder.setPosition(1);
        }
        if (gamepad1.dpad_down) {
            holder.setPosition(0);
        }
    }

    public void armTravel() { // controls arm WORKS


        if (gamepad1.y) {
            if (armPos == 0) {
                arm.setTargetPosition(-2268);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(-.9);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPos = 1;
                clawState = true;
                clawmove.reset();
            }
            if (armPos == 0.5) {
                arm.setTargetPosition(-756);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(-.9);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPos = 1;
                clawState = true;
                clawmove.reset();
            }
        }

        if (gamepad1.x) {
            if (armPos == 1) {
                clawmove.reset();
                claw.setPosition(1);
                clawReady = true;
                clawState = false;

            }

        }
        if (gamepad1.b) {
            if (armPos == 0) {
                arm.setTargetPosition(-1512);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(-.9);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPos = 0.5;
                clawState = true;
            }
        }



        if(clawState && clawmove.milliseconds() > 1000)

    {
        claw.setPosition(0);
    }
        if(clawReady)

    {
        if (clawmove.milliseconds() > 800) {
            arm.setTargetPosition(2268);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setPower(.9);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armPos = 0;
            armmove = false;
            clawReady = false;
        }
    }

}


    public boolean checkB()
    {

        if (currentB) previousB = true;
        else previousB = false;
        currentB = gamepad1.b;


        if(currentB && !previousB){
            return true;
        }
        return false;
    }

    public boolean checkRB()
    {

        if (currentRB) previousRB = true;
        else previousRB = false;
        currentRB = gamepad1.right_bumper;


        if(currentRB && !previousRB){
            return true;
        }
        return false;
    }
//    public void powerShots(){ //needs testing
//        if(gamepad1.start){
//            //use bettermovebot if possible. WIP
//            moveBot(1,2,2,1,44,.6,false); //will be to better movement updated later
//            shooter.setPower(-shooterPower);
//            flicker.setPosition(0);
//            flicker.setPosition(0.7);
//            shooter.setPower(0);
//            moveBot(1,2,2,1,8,.6,false); //will be updated to better movement later
//            shooter.setPower(-shooterPower);
//            flicker.setPosition(0);
//            flicker.setPosition(0.7);
//            shooter.setPower(0);
//            moveBot(1,2,2,1,8,.6,false); //will be updated to better movement later
//            shooter.setPower(-shooterPower);
//            flicker.setPosition(0);
//            flicker.setPosition(0.7);
//            shooter.setPower(0);
//        }
//    }

//    public void moveBot(int leftT, int rightT, int leftB, int rightB, int distance, double power, boolean withIntake){
//        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
//        //turnBot(2, 2, 1, 1, -24, .60); //Backward
//        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
//        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
//        if (leftT == 1) {
//            leftFront.setDirection(DcMotor.Direction.FORWARD);
//        } else if(leftT == 2){
//            leftFront.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (leftB == 1) {
//            leftBack.setDirection(DcMotor.Direction.FORWARD);
//        } else if(leftB == 2){
//            leftBack.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (rightT == 2) {
//            rightFront.setDirection(DcMotor.Direction.FORWARD);
//        } else if(rightT == 1){
//            rightFront.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (rightB == 2) {
//            rightBack.setDirection(DcMotor.Direction.FORWARD);
//        } else if(rightT == 1){
//            rightBack.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        /*if(withIntake) {
//            while (shooterTime.milliseconds() <= 5000) {
//                intake.setPower(.5);
//                transfer.setPower(1);
//                heartbeat();
//            }
//        }*/
//
//        //Moves the robot
//        int travel = (int)(distance * TPI);
//        for (DcMotorEx motor : motors) {
//            motor.setTargetPosition(travel);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setPower(power);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        //intake.setPower(0);
//        // transfer.setPower(0);
//    }
// public double currentAngle() {
// return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
// }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.Camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Timer;
//import gayness
@Autonomous
public class AutoMain extends LinearOpMode {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack, arm, transfer, intake, shooter;
    private Servo claw, flicker, holder;
    private DcMotorEx[] motors;
    private final double TPI = 33.5625;
    private int scenario;
    OpenCvInternalCamera phoneCam;
    AutoMain.UltimateGoalDeterminationPipeline pipeline;
    private int distance;

    @Override
    public void runOpMode() throws InterruptedException  { //Load Zone B
        initialize();
        //moveBot(1,1,2,2,1, .6,true);
        moveBot(1,1,1,1,60, .6, true); //forward
        moveBot(1,2,2,1,8, .6, true); //strafe right
        yeetRing();
        if(scenario == 0) //zone A
        {
            moveBot(1, 1, 1, 1, 15, .6, true); //forward
            moveBot(2, 1, 2, 1, -16, .6, true); //turn
            armTravel();
        }
        if(scenario == 1) { //zone B
            moveBot(1, 1, 1, 1, 18, .6, true); //forward0
            moveBot(1,2,2,1,18, .6, true); //strafe right
            armTravel();
        }
        if(scenario == 4){ //zone C
            moveBot(1,2,2,1,-11, .6, true); //strafe left
            moveBot(1,2,1,2,1,.6,true);
            moveBot(1, 1, 1, 1, 48, .6, true); //forward
            armTravel();
            moveBot(1, 1, 1, 1, -36, .6, true); //backward
        }
        claw.setPosition(1);


        //0 Void 1 Forward 2 Reverse
        //moveBot(1,1,2,3,18, .6, true);'
        //if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.NONE)
        // moveBot(1,1,2,3,12, .6, true);
        // else if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.ONE)
        // moveBot(1,1,2,3,24, .6, true);
        // else if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.FOUR)
        // moveBot(1,1,2,3,48, .6, true);
        //moveBot(1, 1, 2, 2, 48, .60, true); //forward
        //f();
        //moveBot(1, 1, 2, 2, 12, .60, true); //forward
        //setDownWobbler();
        //activate flicker



    }

    public void initialize() {
        //Even if I'm not using it, I have to map it because it is mapped on the bot.
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new AutoMain.UltimateGoalDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        claw.setPosition(1);
        flicker.setPosition(.7);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(() ->{
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });
        while(!opModeIsActive()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            scenario = 1;
            if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.NONE) //zone A
            {
                scenario = 0;
            }
            if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.ONE) //zone A
            {
                scenario = 1;
            }
            if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.FOUR) //zone A
            {
                scenario = 4;
            }
            sleep(50);
        }
        motors = new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motor == leftFront || motor == leftBack)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
        waitForStart();
    }
//from retard import idiot
    /**
     * @param power
     * @param distance inchies so you will have to convert to tics
     */

    //This method accepts 6 variables:
    // 4 of them are motors (leftFront, leftBack, rightFront, rightBack)
    //These variables will accept 3 integers: 0 Void, 1 Forward, 2 Reverse
    //It also accepts power and distance

    //withIntake will be used later (work in progress)
    public void betterMoveBot(double angleRadians, double power, int distance, double turnAmount, boolean withIntake) throws InterruptedException{ //turnAmount is + or - depending on right or left, in range -1 to 1
        //testing needed for how much distance = 90 degrees
        //estimated total x + total y movement distance needed in distance variable for angled slides (needs testing)
        double powerAngle = angleRadians - (Math.PI / 4); // conversion for correct power values
        double[] motorPowers = new double[4];//left front, left back, right front, right back
        motorPowers[0] = Range.clip(0.5 * Math.cos(powerAngle)- turnAmount, -1.0, 1.0);
        motorPowers[1] = Range.clip(0.5 * Math.sin(powerAngle) - turnAmount, -1.0, 1.0);
        motorPowers[2] = Range.clip(0.5 * Math.sin(powerAngle) + turnAmount, -1.0, 1.0);
        motorPowers[3] = Range.clip(0.5 * Math.cos(powerAngle) +turnAmount, -1.0, 1.0);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        int travel = (int)(distance * TPI);
        for (int i = 0;i<4;i++) {
            motors[i].setTargetPosition(travel);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setPower(motorPowers[i]*(power/2));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            heartbeat();
        }
    }
    public void moveBot(int leftT, int rightT, int leftB, int rightB, int distance, double power, boolean withIntake) throws InterruptedException{
        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
        //turnBot(2, 2, 1, 1, -24, .60); //Backward
        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
        if (leftT == 1) {
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        } else if(leftT == 2){
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftB == 1) {
            leftBack.setDirection(DcMotor.Direction.FORWARD);
        } else if(leftB == 2){
            leftBack.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightT == 2) {
            rightFront.setDirection(DcMotor.Direction.FORWARD);
        } else if(rightT == 1){
            rightFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightB == 2) {
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        } else if(rightT == 1){
            rightBack.setDirection(DcMotor.Direction.REVERSE);
        }

        /*if(withIntake) {
            while (shooterTime.milliseconds() <= 5000) {
                intake.setPower(.5);
                transfer.setPower(1);
                heartbeat();
            }
        }*/

        //Moves the robot
        int travel = (int)(distance * TPI);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(travel);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //This is what checks if the motors are supposed to be still running.
        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            heartbeat();
        }
        //intake.setPower(0);
        // transfer.setPower(0);
    }

    public void yeetRing() throws InterruptedException { //NEEDS TO BE REVAMPED TO INCLUDE FLICKER AND PARAMETER FOR RINGS
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(-.75);
        ElapsedTime shooterTime = new ElapsedTime();
        int flick = 3000;
        //intake.setPower(-.5);
        //transfer.setPower(1);
        while (shooterTime.milliseconds() <= 10000) {
            flick = (int)shooterTime.milliseconds() + flick;
            if(shooterTime.milliseconds() >= 2000)
                while (shooterTime.milliseconds() <= flick)
                    flicker.setPosition(0);
            flick = 850;
            flick = (int)shooterTime.milliseconds() + flick;
            while(shooterTime.milliseconds() <= flick)
                flicker.setPosition(.7);
            flick = 850;
        }
        flicker.setPosition(.7);
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
    public void armTravel() throws InterruptedException {
        ElapsedTime wobbler = new ElapsedTime();
        arm.setTargetPosition(-2268);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(wobbler.milliseconds() <= 3000){
            heartbeat();
            if(!arm.isBusy())
            {
                claw.setPosition(0);
            }
        }
        arm.setTargetPosition(2268);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(arm.isBusy()){
            heartbeat();
        }
        arm.setPower(0);

    }





    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition{
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90, 80);

        static final int REGION_WIDTH = 10;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;

        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            if(avg1 > FOUR_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            } else if(avg1 > ONE_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.ONE;
            } else{
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
        public RingPosition getPosition() {
            return position;
        }


    }

}

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoMain;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;



public class UltiamteGoalDeterminationPipeline extends OpenCvPipeline
{

    public enum Rings
    {
        ZERO,
        ONE,
        FOUR
    }

    private static final Point box = new Point(180, 100);
    private static final Scalar green = new Scalar(0, 255, 0);
    private static final int width = 40, height = 50;
    private static final int rings4 = 150, rings1 = 120;
    private int avg, scenario;
    private Point pointA;
    private Point pointB;
    private Mat YCrCb;
    private Mat Cb;
    private Mat regional;
    public volatile Rings position;

    @Override
    public Mat processFrame(Mat input) {
        initialize();
        tonOfConversions(input);
        Imgproc.rectangle(input, pointA, pointB, green, 2);
        determineRings();
        Imgproc.rectangle(input, pointA, pointB, green, -1);
        return input;
    }

    public void initialize()
    {
        pointA = new Point(box.x, box.y);
        pointB = new Point(box.x + width, box.y + height);
        YCrCb = new Mat();
        Cb = new Mat();
        regional = Cb.submat(new Rect(pointA, pointB));
    }

    //This method converts to a file type that is readable by OpenCV.
    //Then, it uses channels to figure out how far the image is
    //It also figures out what percentage of the box is the color needed.
    //The higher the value, the closer the color to the color of the ring.
    public void tonOfConversions(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
        avg = (int) Core.mean(regional).val[0];
    }

    public void determineRings() {
        if (avg > rings4) {
            position = Rings.FOUR;
            scenario = 4;
        } else if (avg > rings1) {
            position = Rings.ONE;
            scenario = 1;
        } else {
            position = Rings.ZERO;
            scenario = 0;
        }
    }

    public int getScenario()
    {
        return scenario;
    }

    public int getAnalysis(){return avg;}
}
package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VuMark;
import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;

import static android.R.attr.hardwareAccelerated;
import static android.R.attr.process;
import static com.sun.tools.javac.main.Option.O;
import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.robotutil.DriveTrain.Direction.*;
import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.HoughLines;
import static org.opencv.imgproc.Imgproc.line;

/**
 * Created by pranav on 1/11/18.
 */
@Autonomous
public class VisionTesting1 extends ManualVisionOpMode {

    /**
     * Returns every frame an image received from the camera.
     * If your method runs for too long, frames will be skipped (this is normal behaviour, reducing
     * FPS).
     *
     * @param rgba RGBA image
     * @param gray Grayscale image
     * @return Return an image to draw onto the screen
     */

    RelicRecoveryVuMark vuMark;
    private boolean strafeleft;
    private boolean strafeRight;
    private boolean alignWithCryptoCol;
    private int detectedLines;

    static double jewelArmInitPosition = 0, jewelArmDownPos = 0.96, jewelArmUpPos = 0.3;
    static Servo jewelArm;
    Servo dump;
    boolean red = false;
    Servo hitJewel;
    ColorSensor jewelColor;
    MRColorSensor cryptoColor;//add to config
    MRColorSensor colorSensor;

    int startingPos = 0; //0 IS CORNER POSITION, 1 IS SANDWITCH POSITION

    VuMark vm;

    private DriveTrain driveTrain;
    private VisionTesting1.RelicOpModeHelper helper = null;
    private ExecutorService executorService = null;

    private volatile boolean isStarted = false;
    private volatile boolean stopRequested = false;



    public VisionTesting1() {
        //initialize();
        driveTrain = new DriveTrain();
       /* DcMotor rF = this.hardwareMap.dcMotor.get("rF");
        DcMotor rB= this.hardwareMap.dcMotor.get("rB");
        DcMotor lF = this.hardwareMap.dcMotor.get("lF");
        DcMotor lB =this.hardwareMap.dcMotor.get("lB");
        DcMotor rIntake = this.hardwareMap.dcMotor.get("rIntake");
        DcMotor lIntake = this.hardwareMap.dcMotor.get("lIntake");
        DcMotor rSlide = this.hardwareMap.dcMotor.get("rSlide");
        DcMotor lSlide = this.hardwareMap.dcMotor.get("lSlide");
        BNO055IMU adaImu = this.hardwareMap.get(BNO055IMU.class, "imu");
        ColorSensor jewelColor = hardwareMap.colorSensor.get("jewelColor");
        MRColorSensor colorSensor = new MRColorSensor(jewelColor);
      //  driveTrain = new DriveTrain( rF,  rB,  lF,  lB,  rIntake,  lIntake,  rSlide,  lSlide, adaImu,jewelColor, colorSensor );*/
    }

    @Override
    public void init() {
        super.init();
        this.executorService = ThreadPool.newSingleThreadExecutor(this.getClass().getSimpleName());
        this.helper = new VisionTesting1.RelicOpModeHelper();
        this.isStarted = false;
        this.stopRequested = false;
        this.executorService.execute(this.helper);

    }

    @Override
    public void stop(){
        openCVCamera.disableView();
        openCVCamera.disconnectCamera();
        driveTrain.opmodeDeActivated();
    }

    @Override
    public void start(){
        driveTrain.opmodeActivated();
    }
    public  Mat frame(Mat rgba, Mat gray) {
        /*
        Detect objects from rgba or grayscale matrix
        Include commands to control robot.
        If you are not displaying anything then just return null.
        This method will be called for each frame. You need to set instance variables to keep
        the state. Most likely another thread will call this method.
        */
        Log.d(VisionTesting1.class.getName(), "frame called");
        Log.d(VisionTesting1.class.getName(), "size " + rgba.size());
        if(strafeleft) {
            double[] vals = lineDetect(rgba);
            driveTrain.StrafeImuCrypto(LEFT, .2, 5, this.telemetry, vals);

        }
        if(strafeRight){
            double[] vals = lineDetect(rgba);
            driveTrain.StrafeImuCrypto(RIGHT, .2, 5, this.telemetry, vals);

        }
        if(alignWithCryptoCol) {
            switch (vuMark) {
                case RIGHT:

            }
        }
        telemetry.addData("frame size  -->  ", rgba.cols() + "  "+ rgba.rows());
        return null;
    }

    public double[] lineDetect(Mat RGB){

        double [] vals = new double [10];
        //List <Double> list = new ArrayList<Double>();
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat res = new Mat();
        //Mat original = Imgcodecs.imread("/home/pranav/Desktop/Crypto.jpg");
        Mat original = RGB;
        Scalar lower_red = new Scalar(0,1,1);
        Scalar upper_red = new Scalar(10,255,255);
        inRange(hsv, lower_red, upper_red,mask);
        bitwise_and(original,original,res ,mask );
       /* Imgcodecs.imwrite("/tmp/frame.jpg", original);
        Imgcodecs.imwrite("/tmp/mask.jpg", mask);
        Imgcodecs.imwrite("/tmp/res.jpg", res);*/
        Mat source = new Mat();
        Imgproc.cvtColor( original, source, Imgproc.COLOR_BGR2GRAY);
        /*Imgcodecs.imwrite("/tmp/crypto.jpg", source);
        Imgcodecs.imwrite("/tmp/blur.jpg", source);*/
        Mat vector = new Mat();
        HoughLines(mask, vector, 3, PI/5, 200, 1, 1,0, PI/9);
        //HoughLinesP(mask, vector, 1, PI/180, 80, 30, 10);
        for (int i = 0; i < vector.rows(); i++) {
            double data[] = vector.get(i,0 );
            double rho1 = data[0];
            //System.out.println(rho1);
            vals[i] = rho1;

            double theta1 = data[1];
            double cosTheta = Math.cos(theta1);
            double sinTheta = Math.sin(theta1);
            double x0 = cosTheta * rho1;
            double y0 = sinTheta * rho1;
            Point pt1 = new Point(x0 + 10000 * (-sinTheta), y0 + 10000 * cosTheta);
            Point pt2 = new Point(x0 - 10000 * (-sinTheta), y0 - 10000 * cosTheta);
            Imgproc.line(original, pt1, pt2, new Scalar(0, 0, 255), 2);

        }
        System.out.println("rows " + vector.rows() + "  "+ "columns " + vector.cols() );
        //Imgcodecs.imwrite("/tmp/withLines.jpg", original);

        return vals;
        //return vals;
        /*
           Imgproc.threshold(source, source, 127, 255, THRESH_BINARY);
           Mat skel = new Mat(source.size(), CV_8UC1, new Scalar(0));
           Mat temp = new Mat(source.size(), CV_8UC1);
           Mat element = getStructuringElement(MORPH_CROSS, new Size(3, 3));
           boolean done;
           do{
               morphologyEx(source, temp, MORPH_OPEN, element);
               bitwise_not(temp, temp);
               bitwise_and(source, temp, temp);
               bitwise_or(skel, temp, skel);
               erode(source, source, element);
               double max;
               MinMaxLocResult minMx = minMaxLoc(source);
               System.out.println(minMx.maxLoc);
               done = (minMx.maxVal == 0 );
           }while(!done);
           Imgcodecs.imwrite("/tmp/skel.jpg", skel);*/

    }


    private void moveRobot(){

    }
    protected class RelicOpModeHelper implements Runnable {
        protected RuntimeException exception = null;
        protected boolean isShutdown = false;

        public RelicOpModeHelper() {
        }

        public void run() {
            ThreadPool.logThreadLifeCycle("VisionTesting1 main", new Runnable() {
                public void run() {
                    VisionTesting1.RelicOpModeHelper.this.exception = null;
                    VisionTesting1.RelicOpModeHelper.this.isShutdown = false;

                    try {
                        VisionTesting1.this.runOpMode();
                        VisionTesting1.this.requestOpModeStop();
                    } catch (InterruptedException var7) {
                        RobotLog.d("VisionTesting1 received an InterruptedException; shutting down this op mode");
                    } catch (CancellationException var8) {
                        RobotLog.d("VisionTesting1 received a CancellationException; shutting down this op mode");
                    } catch (RuntimeException var9) {
                        VisionTesting1.RelicOpModeHelper.this.exception = var9;
                    } finally {
                        TimestampedI2cData.suppressNewHealthWarningsWhile(new Runnable() {
                            public void run() {
                                if(VisionTesting1.this.telemetry instanceof TelemetryInternal) {
                                    VisionTesting1.this.telemetry.setMsTransmissionInterval(0);
                                    ((TelemetryInternal)VisionTesting1.this.telemetry).tryUpdateIfDirty();
                                }

                            }
                        });
                        VisionTesting1.RelicOpModeHelper.this.isShutdown = true;
                    }
                }
            });
        }

        public boolean hasRuntimeException() {
            return this.exception != null;
        }

        public RuntimeException getRuntimeException() {
            return this.exception;
        }

        public boolean isShutdown() {
            return this.isShutdown;
        }
    }

    void runOpMode()throws InterruptedException{
        telemetry.addData("Status", "= Initialized");
        telemetry.update();
        //initialize();
        while(driveTrain.opmodeIsActive == false){

            telemetry.addData("Status  -->", "  waiting for start");
            telemetry.update();
        }
        telemetry.addData("Status -->", "  started");
        telemetry.update();
        Functions.waitFor(1000000);
        if(startingPos == 0){
            if(red){
                /*
                driveTrain.hitJewel(red);
                driveTrain.imuDriveCryptoGraph(.3, 2, this.telemetry);
                RelicRecoveryVuMark vuMark = vm.detectColumn(3);
                this.vuMark = vuMark;
                driveTrain.encoderDrive(.3, 5, FORWARD, 4);*/
                strafeLeftToCrypto(5000);
                switch (vuMark) {
                    case CENTER:
                        driveTrain.encoderDrive(.4, 5, FORWARD, 3);
                        driveTrain.dumpBlock();
                    case RIGHT:

                }

            }
            if(!red){
                driveTrain.hitJewelMomentum(.2, 3, 18, red);



            }

        }

    }

    public void initialize(){
        DcMotor rF, rB, lF, lB, rIntake, lIntake, rSlide, lSlide;
        BNO055IMU adaImu;

        lB = hardwareMap.dcMotor.get("lB");
        rF = hardwareMap.dcMotor.get("rF");
        lF = hardwareMap.dcMotor.get("lF");
        rB = hardwareMap.dcMotor.get("rB");

        rSlide = hardwareMap.dcMotor.get("rSlide");
        lSlide = hardwareMap.dcMotor.get("lSlide");
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");

        IMU imu = new IMU(adaImu);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.FORWARD);
        lB.setDirection(DcMotor.Direction.REVERSE);
        lF.setDirection(DcMotor.Direction.REVERSE);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        lIntake = hardwareMap.dcMotor.get("lIntake");
        rIntake = hardwareMap.dcMotor.get("rIntake");
        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        colorSensor = new MRColorSensor(jewelColor);
        ColorSensor cColor = hardwareMap.colorSensor.get("cryptoColor");//add to config
        cryptoColor = new MRColorSensor(cColor);
        hitJewel = hardwareMap.servo.get("hitJewel"); //add to config
        dump = hardwareMap.servo.get("dump"); //add to config





        driveTrain = new DriveTrain( rF,  rB,  lF,  lB,  rIntake,  lIntake,  rSlide,  lSlide, adaImu, jewelColor, colorSensor, cryptoColor, hitJewel, dump);
    }

    public void strafeLeftToCrypto(long timeout) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeout) {
            if(!strafeleft) {
                strafeleft = true;
            }
        }
        strafeleft = false;
    }
    public void strafeRightToCrypto(long timeout) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeout) {
            if(!strafeRight) {
                strafeRight = true;
            }
        }
        strafeRight= false;
    }
    public void alignWithCryptoCol(long timeout){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeout) {
            if(!alignWithCryptoCol) {
                alignWithCryptoCol = true;
            }
        }
        alignWithCryptoCol= false;


    }
    public void options(){
        telemetry.addData("Team", "Blue");
        telemetry.update();
        boolean confirmed = false;
        red = false;
        while(!confirmed){
            if (gamepad1.a){
                red = true;
                colorSensor.team = Team.RED;

                telemetry.addData("Team", red ? "Red": "Blue");
            }
            if (gamepad1.b){
                red = false;
                colorSensor.team = Team.BLUE;

                telemetry.addData("Team", red ? "Red": "Blue");
            }
            if (gamepad1.x){
                startingPos = 0;

                telemetry.addData("Starting Position", startingPos ==0 ? "corner": "sandwich");
            }
            if (gamepad1.y){
                startingPos = 1;

                telemetry.addData("Starting Position", startingPos ==0 ? "corner": "sandwich");
            }

            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Team", red ? "Red" : "Blue");
                telemetry.addData("Starting Position", startingPos ==0 ? "corner": "sandwich");
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                confirmed = true;
            }

        }
    }
}

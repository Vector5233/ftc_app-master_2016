package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.ContextWrapper;
import android.content.pm.ApplicationInfo;
import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;

//@TeleOp(name="ImageGrabberTeleop", group = "myGroup")

/**
 * Created 2017.02.06 by Jeff Cagle
 *
 * This teleop allows the driver to save a picture of the beacon along with a 16-region (4x4)
 * color analysis of the top half of the picture.
 *
 */

public class ImageGrabberTeleop extends OpMode {

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate, bopper;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    // Strafing variables
    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    // H_C_S scales how fast the motors correct rotation when strafing, F_F compensates for further backwards motion of the entire strafing bot
    final double HEADING_CORRECTION_SCALE = 10, FUDGE_FACTOR = 1.10;


    // Flicker targeting variables
    //boolean targetSet = false;
    int encVal, targetVal;
    int FLICKER_ERROR = 184; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181

    // Start a timer, for any time-related functions
    public ElapsedTime runTime = new ElapsedTime();
    double timeMark = 0;

    // Vuforia window bounds (Area checked for red/blue)
    int WINDOW_X_MIN = 600, WINDOW_X_MAX = 1200;
    int WINDOW_LEFT_Y = 719, WINDOW_MID_Y = 360, WINDOW_RIGHT_Y = 0;

    // Vuforia misc.
    int leftRedAvg = 0, leftBlueAvg = 0, rightRedAvg = 0, rightBlueAvg = 0;
    int leftRed = 0, leftBlue = 0, rightRed = 0, rightBlue = 0;
    //int c = 0, red = 0, blue = 0;

    static long numImages = 0;
    static String grabberStatus = "done";

    Image rgb = null;
    Bitmap bm = null;
    boolean imageFound = false;
    int imageCount=0;

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    VuforiaTrackableDefaultListener myListener;

    VuforiaLocalizer locale;
    VuforiaTrackables beacons;

    double driveEncAvg = 0;

    // Initialize robot before match starts
    public void init() {

        // Map this program's names for motors to the RC Phone's config file
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        flicker = hardwareMap.dcMotor.get("flicker");
        collector = hardwareMap.dcMotor.get("collector");
        gate = hardwareMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Reverse motors that are physically reversed on bot (so that setPower(1) means "forward" for every motor)
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset drive encoders
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset flicker encoder to 0, then set it to run using encoders
        flicker.setDirection(DcMotor.Direction.REVERSE);
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize flicker's target value for one rotation to be its current position, so that it doesn't start flicking if the reset failed (for some reason)
        targetVal = flicker.getCurrentPosition();

        // Set gate to be closed at the start of the match
        gate.setPosition(0);

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();

        timeMark = runTime.time();

        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5) {

        }

        // Initialize Vuforia
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARVTSuz/////AAAAGRYwaVCcd060r5V5zPNagLKBBcKXizn4VpPEd3T4xIEYhTInQqa9AAn56zmC0i9zNPFczizZ6vg6qZsgFikzbzyDfGkU7xTtM+zdwJ6rrBTVVdHAojTCcwKdFAJ3NFgSBamYM3I2jQqXjE6CxgRJkUDkXa6G2pO2ZOtT2/5JN4s55X9osscF1jeMNkndUrXU4P3dGHbVErHDsTMDd8piqvMLhvCYDLHEKHuvMetrEp/5FJYSDSckMWc6Zffd3rNQYxN0htcDYXf8i6y/yOtyknYiKo4ck5gsgdd/JIQ8X5xhG52bMB9cgVzwD2sJdk1veyY2zBbAxdQNW4PgRlx31jJ0RJZA7ODkHbmcgR2pI1dH";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        this.locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        this.beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        beacons.activate();

        telemetry.addLine("Teleop Initialized!");

    }

    public void loop() {


        telemetry.addData(">", "Robot Heading Z = %d", gyro.getIntegratedZValue());
        telemetry.addLine("imageGrabber: "+grabberStatus);
        // Drive Controls //

        frontRight.setPower(gamepad1.right_stick_y);
        backRight.setPower(gamepad1.right_stick_y);
        frontLeft.setPower(gamepad1.left_stick_y);
        backLeft.setPower(gamepad1.left_stick_y);

        // Gate Controls //

        /*// Opens gate when x is pressed
        if (gamepad2.x) {
            gate.setPosition(.3); // Last set at .3
            gateState = "Open";
        }
        // Closes gate by default
        else {
            gate.setPosition(0); // Last set at 0
            gateState = "Closed";
        }

        // Collector Controls //

        // Collector out when dpad_up is pressed
        if (gamepad2.dpad_up) {
            collector.setPower(1);
        }
        // Collector in when dpad_down is pressed
        else if (gamepad2.dpad_down) {
            collector.setPower(-1);
        }
        // Collector off by default
        else {
            collector.setPower(0);
        }


        // Flicker Controls //

        //Working note: flicker.setPower(-1) is the right direction

        if (gamepad2.y && flicker.getPower() == 0) {
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360;

            flicker.setTargetPosition((targetVal - FLICKER_ERROR));
            timeMark = runTime.time();
            flicker.setPower(1);
        }

        if ((runTime.time() - timeMark) >= 2) {
            flicker.setPower(0);
        }
*/
        // Vuforia Magic


        // Get translation and degree values from phone to Vuforia printout images under beacons
        /*for (VuforiaTrackable beac : beacons) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

            if (pose != null) {

                //Matrix34F rawPose = new Matrix34F();
                //float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(2), translation.get(0)));
                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
            }
        }*/

        /*for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                imageFound = true;

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);
            }
            else {
                imageFound = false;
            }
        }*/


        if (gamepad1.a) {
            grabFrame();
            ProcessFrame p = new ProcessFrame();
            p.run();
        }



    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
        headingSet = true;
    }







    // Grab a frame from the phone's camera, for use in color analyzation
    public void grabFrame() {
        VuforiaLocalizer.CloseableFrame frame;

        if (grabberStatus.equals("busy")) {
            return;
        }

        try {
            frame = locale.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch (InterruptedException e) {
            frame = null;
        }

        if (frame != null) {
            numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }//if
            }//for

            if (rgb != null) {
                bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(rgb.getPixels());
            }
            grabberStatus = "captured";
        }
    }

    private class ProcessFrame extends Thread {
        public void run() {
            final int dx = (WINDOW_X_MAX - WINDOW_X_MIN)/4;
            final int dy = (WINDOW_LEFT_Y - WINDOW_RIGHT_Y)/4;
            final int[] windowx = {WINDOW_X_MIN, WINDOW_X_MIN+dx,
                    WINDOW_X_MIN+2*dx, WINDOW_X_MIN + 3*dx };
            final int[] windowy = {WINDOW_RIGHT_Y, WINDOW_RIGHT_Y+dy,
                    WINDOW_RIGHT_Y + 2*dy, WINDOW_RIGHT_Y + 3*dy};
            int basex, basey, i, j;
            long total;

            if (grabberStatus.equals("busy")) {
                return;
            }

            grabberStatus = "busy";
            ContextWrapper cw = new ContextWrapper(hardwareMap.appContext);
            // path to /data/data/yourapp/app_data/imageDir
            File directory = cw.getDir("Beacon Images", Context.MODE_PRIVATE);
            // Create imageDir
            PrintWriter outputStream;
            try {
                outputStream = new PrintWriter(directory + "Beacon" + String.valueOf(imageCount) + ".txt");

            } catch (Exception e) {
                grabberStatus = "failed";
                return;
            }

/*
// Hey! Wil here. This block had errors and AS won't let me download anything until ALL programs
// are error-free. Consider it debugged.

            try {
                for (basex : windowx) {
                    for (basey : windowy) {
                        total = 0;
                        for (i = basex; i < basex+dx; i++) {
                            for (j = basey; j < basey+dy; j++) {
                                total += Long.valueOf(bm.getPixel(i,j));
                            }
                        }
                        total = total / (dx*dy);
                        outputStream.write("("+Integer.toString(basey)+","+Integer.toString(basex)+")\n");
                        outputStream.write(Long.toHexString(total) + "\n\n");

                    }
                }
                outputStream.close();
                grabberStatus = "done";
            } catch (InterruptedException) {
                grabberStatus="failed";
            }*/

        }
    }

    public String saveImage(Bitmap b) {

        ContextWrapper cw = new ContextWrapper(hardwareMap.appContext);
        // path to /data/data/yourapp/app_data/imageDir
        File directory = cw.getDir("Beacon Images", Context.MODE_PRIVATE);
        // Create imageDir
        File mypath=new File(directory,"Beacon" + String.valueOf(imageCount) + ".png");

        FileOutputStream fos = null;
        try {
            fos = new FileOutputStream(mypath);
            // Use the compress method on the BitMap object to write image to the OutputStream
            b.compress(Bitmap.CompressFormat.PNG, 100, fos);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                fos.close();
                imageCount++;
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return directory.getAbsolutePath();
    }
        // Find average amount of red and blue on left phone window

    public void checkCameraLeft() {
        leftRed = 0;
        leftBlue = 0;

        // Check all pixels in defined window sizes, and add up amount of red and blue in each
        for (int x = WINDOW_X_MIN; x < WINDOW_X_MAX; x++) {

            for (int y = WINDOW_MID_Y; y < WINDOW_LEFT_Y; y++) {
                leftRed += ((bm.getPixel(x, y)) & 0x00FF0000) >> 16;
                leftBlue += ((bm.getPixel(x, y)) & 0x000000FF);
            }

        }


        // Avg variables equal the amount of red/blue (leftRed/leftBlue) divided by the number of pixels scanned
        leftRedAvg = leftRed / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_LEFT_Y - WINDOW_MID_Y));
        leftBlueAvg = leftBlue / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_LEFT_Y - WINDOW_MID_Y));
    }

    // Find average amount of red and blue on right phone window
    public void checkCameraRight() {
        rightRed = 0;
        rightBlue = 0;

        // Check all pixels in defined window sizes, and add up amount of red and blue in each
        for (int x = WINDOW_X_MIN; x < WINDOW_X_MAX; x++) {

            for (int y = WINDOW_RIGHT_Y; y < WINDOW_MID_Y; y++) {
                rightRed += ((bm.getPixel(x, y)) & 0x00FF0000) >> 16;
                rightBlue += ((bm.getPixel(x, y)) & 0x000000FF);
            }

        }

        // Avg variables equal the amount of red/blue (rightRed/rightBlue) divided by the number of pixels scanned
        rightRedAvg = leftRed / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));
        rightBlueAvg = leftBlue / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));
    }


}
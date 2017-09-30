
/**
 * Created Jan 15, 2017 by Wil Orlando
 *
 * This is a secondary autonomous for red team, for use alongside a partner.
 *
 * It starts three tiles from the corner vortex, hence the name.
 *
 * This program makes use of Vuforia, along with the RC phone's camera, to find and process beacons
 * on the field.
 *
 * It scores 60 points overall: two 15-pt particles in the center vortex, and one 30-pt beacon.
 */

import android.graphics.Bitmap;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vec2F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.Arrays;

//@Autonomous(name="VopperThreeTileRed", group="MyGroup")
public class VopperThreeTileRed extends LinearOpMode{

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    // Drive variables
    int initialHeading;
    double headingDiffPercent;
    boolean headingSet = false;
    final double HEADING_CORRECTION_SCALE = 3, FUDGE_FACTOR = 1.10;

    // Turn variables
    int startingDegrees = 0;
    int TURN_DEG_TOLERANCE = 1;
    double TURN_SENSITIVITY = .05/(double)TURN_DEG_TOLERANCE;

    // Flicker targeting variables
    int encVal, targetVal;
    int FLICKER_ERROR = 184; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181

    // Start a timer, for any time-related functions
    public ElapsedTime runTime = new ElapsedTime();
    double timeMark = 0;

    // Vuforia window bounds (Area checked for red/blue)
    int WINDOW_X_MIN = 600, WINDOW_X_MAX = 1200;
    int WINDOW_LEFT_Y = 719, WINDOW_MID_Y = 360, WINDOW_RIGHT_Y = 0;

    // Vuforia image alignment
    double X_ALIGNMENT_TOLERANCE = 3;
    double X_ALIGNMENT_SENSITIVITY =.001;
    boolean imageFound = false;
    double ALIGN_MIN_POWER = 0.10;


    // Vuforia color configuration check
    int leftRedAvg = 0, leftBlueAvg = 0, rightRedAvg = 0, rightBlueAvg = 0;
    int leftRed = 0, leftBlue = 0, rightRed = 0, rightBlue = 0;
    enum State {BLUE_RED, RED_BLUE}
    State colorConfig;

    // Vuforia misc.
    long numImages = 0;
    Image rgb = null;
    Bitmap bm = null;

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    VuforiaLocalizer locale;
    VuforiaTrackables beacons;

    double driveEncAvg = 0, leftEncAvg = 0, rightEncAvg = 0;
    String opState = "";


    public void runOpMode() throws InterruptedException {

        initialize();

        telemetrize();

        waitForStart();

        DbgLog.msg("VECTOR- Start Autonomous");

        beacons.activate();

        while(opModeIsActive()) {

            startingDegrees = gyro.getIntegratedZValue();

            op("First drive...");
            driveForTicks(750, .5);
            DbgLog.msg("VECTOR- First Drive Completed");

            op("First turn...");
            turnDegrees(35);
            DbgLog.msg("VECTOR- First Turn Completed");

            op("Flicking...");
            flickOnce();
            DbgLog.msg("VECTOR- First Flick Completed");

            op("Opening Gate...");
            openGate();
            DbgLog.msg("VECTOR- Gate Opened");

            op("Flicking again...");
            flickOnce();
            DbgLog.msg("VECTOR- Second Flick Completed");

            driveForTicks(9000, 1);

            op("Turning to face wall...");
            turnToFaceWall();
            DbgLog.msg("VECTOR- Wall Turn Completed");

            op("Sleeping...");
            sleep(100);
            DbgLog.msg("VECTOR- Sleep Completed");

            op("Aligning with image...");
            encCameraWithImageX(0);
            DbgLog.msg("VECTOR- Image Alignment Completed");

            op("Verifying image found...");
            imageSafety();
            DbgLog.msg("VECTOR- Image Verification Completed");

            op("Checking beacon color...");
            checkColorConfiguration();
            DbgLog.msg("VECTOR- Color Check Completed");

            op("Aligning with button...");
            alignBopperWithButtons();
            DbgLog.msg("VECTOR- Button Alignment Completed");

            op("Turning towards beacon...");
            turnDegrees(90);
            DbgLog.msg("VECTOR- Turn to Beacon Completed");

            op("Hitting beacon...");
            driveToBeacon();
            DbgLog.msg("VECTOR- Hit Beacon");

            op("Done!");
            DbgLog.msg("VECTOR- End Autonomous");
            stop();
        }
    }

    public void initialize() {

        // Map this program's names for motors to the RC Phone's config file
        frontRight = hardwareMap.dcMotor.get ("frontRight");
        frontLeft = hardwareMap.dcMotor.get ("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor. get("backLeft");
        flicker = hardwareMap.dcMotor.get("flicker");
        collector = hardwareMap.dcMotor.get("collector");
        gate = hardwareMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Reverse motors that are physically reversed on bot (so that setPower(1) means "forward" for every motor)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Reset drive encoders
        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_TO_POSITION);

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
            idle();
        }


        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARVTSuz/////AAAAGRYwaVCcd060r5V5zPNagLKBBcKXizn4VpPEd3T4xIEYhTInQqa9AAn56zmC0i9zNPFczizZ6vg6qZsgFikzbzyDfGkU7xTtM+zdwJ6rrBTVVdHAojTCcwKdFAJ3NFgSBamYM3I2jQqXjE6CxgRJkUDkXa6G2pO2ZOtT2/5JN4s55X9osscF1jeMNkndUrXU4P3dGHbVErHDsTMDd8piqvMLhvCYDLHEKHuvMetrEp/5FJYSDSckMWc6Zffd3rNQYxN0htcDYXf8i6y/yOtyknYiKo4ck5gsgdd/JIQ8X5xhG52bMB9cgVzwD2sJdk1veyY2zBbAxdQNW4PgRlx31jJ0RJZA7ODkHbmcgR2pI1dH";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        this.locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        this.beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");


        telemetry.addLine("Autonomous Initialized!");
    }

    // Get average of left drive encoders
    public double getLeftDriveEncAvg() {
        leftEncAvg = (frontLeft.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 2;
        return leftEncAvg;
    }

    // Get average of right drive encoders
    public double getRightDriveEncAvg() {
        rightEncAvg = (frontRight.getCurrentPosition()
                + backRight.getCurrentPosition()) / 2;
        return rightEncAvg;
    }

    // Get average of all drive encoders
    public double getAllDriveEncAvg() {
        driveEncAvg = (frontRight.getCurrentPosition()
                + frontLeft.getCurrentPosition()
                + backRight.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 4;

        return driveEncAvg;
    }

    // Modified encoder average for strafing; Strafing right will yield a positive value, strafing left a negative one
    public double getStrafeEncAvg() {
        driveEncAvg = (frontRight.getCurrentPosition()
                - frontLeft.getCurrentPosition()
                - backRight.getCurrentPosition()
                + backLeft.getCurrentPosition()) / 4;

        return driveEncAvg;
    }


    //set same target for all drive motors
    public void setAllTarget(int target) {
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);
    }

    //return True if any drive motor busy
    public boolean isAllBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }

    // Put telemetry values up on DS screen (for debugging)
    public void telemetrize() {
        telemetry.addData("OpState: ", opState);
        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
        telemetry.addData("Drive Enc Avg: ", driveEncAvg);
        telemetry.addData("startingDegrees:", startingDegrees);

        telemetry.addData("Enc FR: ", frontRight.getCurrentPosition());
        telemetry.addData("Enc FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("Enc BR: ", backRight.getCurrentPosition());
        telemetry.addData("Enc BL: ", backLeft.getCurrentPosition());

        telemetry.addData("leftRedAvg", leftRedAvg);
        telemetry.addData("leftBlueAvg", leftBlueAvg);
        telemetry.addData("FR power", frontRight.getPower());

        telemetry.addData("Color Config: ", colorConfig);
        telemetry.addData("leftBlueAvg: ", leftBlueAvg);
        telemetry.addData("leftRedAvg: ", leftRedAvg);
        telemetry.addData("rightBlueAvg: ", rightBlueAvg);
        telemetry.addData("rightRedAvg: ", rightRedAvg);
        telemetry.update();
    }

    // Wait function that doesn't crash
    public void betterWait(double msec) {
        timeMark = runTime.time();

        msec /= 1000;

        while((runTime.time() < (timeMark + msec)) && opModeIsActive()) {
            telemetry.addData("Time: ", runTime.time());
            telemetry.addData("Target: ", (timeMark + msec));
        }
    }

    // Set left drive motors to a specified power
    public void setLeftDrivePower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    // Set right drive motors to a specified power
    public void setRightDrivePower(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    // Set all drive motors to a specified power
    public void setAllDrivePower (double power) {
        setLeftDrivePower(power);
        setRightDrivePower(power);
    }

    // Stop all drive motors
    public void stopDriveMotors() {
        setAllDrivePower(0);
    }

    // Sets all drive motors to a specified RunMode
    public void setModeAll(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    // Set drive encoders to 0
    public void resetDriveEncoders() {
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Drive straight for a specified number of encoder ticks
    public void driveForTicks(int encTicks, double power) {  // WORKING NOTE: Relying solely on RUN_TO_POSITION right now; if bot doesn't drive straight, modify to incorporate encoders and gyro

        resetDriveEncoders();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);

        initialHeading = gyro.getIntegratedZValue();

        frontLeft.setTargetPosition(encTicks);
        frontRight.setTargetPosition(encTicks);
        backLeft.setTargetPosition(encTicks);
        backRight.setTargetPosition(encTicks);

        while ((Math.abs(getAllDriveEncAvg()) < encTicks) && opModeIsActive()) {
            telemetrize();

            headingDiffPercent = ((double) (gyro.getIntegratedZValue() - initialHeading) / 100.0);

            setRightDrivePower(power - (HEADING_CORRECTION_SCALE * headingDiffPercent));
            setLeftDrivePower(power + (HEADING_CORRECTION_SCALE * headingDiffPercent));

        }

        stopDriveMotors();
    }

    public void setInitialHeading() {
        initialHeading = gyro.getIntegratedZValue();
    }

    // Turn a specified number of degrees (relative to the starting position, using the gyro sensor
    public void turnDegrees(int degrees) {
        int initialHeading = gyro.getIntegratedZValue();
        double power;
        double absDegrees = startingDegrees - gyro.getIntegratedZValue() + degrees;

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(gyro.getIntegratedZValue() - initialHeading - absDegrees) >= TURN_DEG_TOLERANCE && opModeIsActive()) {
            power = (TURN_SENSITIVITY * (double)(gyro.getIntegratedZValue() - initialHeading - absDegrees));
            if (Math.abs(power) < 0.2 && power != 0) {
                power = 0.2 * (power / Math.abs(power));
            }
            else if (Math.abs(power) > 0.4 && power != 0) {
                power = 0.4 * (power / Math.abs(power));
            }


            Range.clip(power, -1, 1);

            setLeftDrivePower(power);
            setRightDrivePower(-power);
            telemetrize();
        }

        stopDriveMotors();
        telemetrize();
        sleep(100);

        superTurn(degrees);
    }

    // More precise turn function, for use in turns that must be within a degree
    public void superTurn (int degrees) {
        int initialHeading = gyro.getIntegratedZValue();
        double power;

        degrees = startingDegrees - gyro.getIntegratedZValue() + degrees;

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(gyro.getIntegratedZValue() - initialHeading - degrees) >= 1 && opModeIsActive()) {
            power = (0.01 * (double)(gyro.getIntegratedZValue() - initialHeading - degrees));
            if (Math.abs(power) < 0.2 && power != 0) {
                power = 0.2 * (power / Math.abs(power));
            }
            else if (Math.abs(power) > 0.4 && power != 0) {
                power = 0.4 * (power / Math.abs(power));
            }


            Range.clip(power, -1, 1);

            setLeftDrivePower(power);
            setRightDrivePower(-power);
            telemetrize();
        }

        stopDriveMotors();
        telemetrize();
        sleep(200);
    }

    // Rotate bot to be parallel with wall, 180 degrees from starting position (because we're red)
    public void turnToFaceWall() {
        turnDegrees (180);
    }

    // Rotate flicker once, to launch a loaded ball
    public void flickOnce() {
        encVal = Math.abs(flicker.getCurrentPosition());
        targetVal = encVal + 3360;

        flicker.setTargetPosition((targetVal - FLICKER_ERROR));
        flicker.setPower(1);

        sleep(2000);

        flicker.setPower(0);
    }

    // Open gate, to load a ball from the hopper into the launcher
    public void openGate() {
        gate.setPosition(.3);

        sleep(1500);

        gate.setPosition(0);
    }

    // Grab a frame from the phone's camera, for use in color analyzation
    public void grabFrame() {
        VuforiaLocalizer.CloseableFrame frame;

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
        }
    }

    // Find average amount of red and blue on left phone window

    public boolean isLeftBlue() {
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

        if (leftBlueAvg > leftRedAvg) {
            return true;
        }
        else {
            return false;
        }
    }

    // Find average amount of red and blue on right phone window
    public boolean isRightBlue() {
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
        rightRedAvg = rightRed / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));
        rightBlueAvg = rightBlue / ((WINDOW_X_MAX - WINDOW_X_MIN) * (WINDOW_MID_Y - WINDOW_RIGHT_Y));

        if (rightBlueAvg > rightRedAvg) {
            return true;
        }
        else {
            return false;
        }
    }

    // Align camera with an x value from the beacon image
    public void alignCameraWithImageX(int targetX) {

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                imageFound = true;

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double xDistance = translation.get(0);
                double alignmentPower;

                telemetry.addData(beac.getName() + "xDistance", xDistance);
                telemetrize();

                while((Math.abs(xDistance - targetX)) > X_ALIGNMENT_TOLERANCE && opModeIsActive()) {

                    pose = myListener.getPose();
                    translation = pose.getTranslation();
                    xDistance = translation.get(0);

                    alignmentPower = Range.clip(((xDistance - targetX) * X_ALIGNMENT_SENSITIVITY), -1, 1);

                    if (Math.abs(alignmentPower) < ALIGN_MIN_POWER && alignmentPower != 0) {
                        alignmentPower = ALIGN_MIN_POWER * (alignmentPower / Math.abs(alignmentPower));
                    }

                    setAllDrivePower(alignmentPower);

                    telemetry.addData(beac.getName() + "xDistance", xDistance);
                    telemetrize();
                }

                break;
            }
        }

        stopDriveMotors();
    }

    public void encCameraWithImageX(int targetX) {
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                imageFound = true;

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);
                double xDistance = translation.get(0);
                telemetry.addData(beac.getName() + "xDistance", xDistance);

                runMM((int)(xDistance - targetX));

                break;
            }
        }

        telemetrize();
        stopDriveMotors();
    }

    // Runs a set number of millimeters on the field (using wheels with a radius of 5 cm)
    public void runMM(int mm) {
        // mm / 1 x 1 rot / 100pi mm x 1120 ticks / 1 rot

        int ticks = (int)((1120 * mm) / (100 * 3.14159));
        double power = ALIGN_MIN_POWER * (ticks / Math.abs(ticks));

        driveForTicks(Math.abs(ticks), power);
    }

    public void imageTest() {
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VuforiaTrackableDefaultListener myListener;

        for (VuforiaTrackable beac : beacons) {
            myListener = ((VuforiaTrackableDefaultListener) beac.getListener());
            OpenGLMatrix pose = myListener.getPose();

            if (pose != null) {

                timeMark = runTime.time();

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "-Translation", translation);

                double xDistance;

                while (runTime.time() - timeMark < 10 && opModeIsActive()){
                    pose = myListener.getPose();
                    translation = pose.getTranslation();
                    xDistance = translation.get(0);
                    telemetry.addData(beac.getName() + "xDistance", xDistance);
                    telemetrize();
                }

                telemetrize();
                sleep(2000);
            }
        }
    }

    public void imageSafety() {
        if (!imageFound) {
            op("Couldn't find image. Stopping robot.");

            stop();
        }
    }

    public void checkColorConfiguration(){
        grabFrame();

        if (!isLeftBlue() && isRightBlue()) {
            colorConfig = State.RED_BLUE;
        }
        else if (isLeftBlue() && !isRightBlue()) {
            colorConfig = State.BLUE_RED;
        }
        else {
            if (leftBlueAvg < rightBlueAvg) {
                colorConfig = State.RED_BLUE;
            }
            else {
                colorConfig = State.BLUE_RED;
            }
        }
    }

    public void alignBopperWithButtons() {
        if (colorConfig == State.BLUE_RED) {
            // We're red, so we're aiming for the left button (from the front)

            driveForTicks(294, -ALIGN_MIN_POWER);

            telemetry.addData("We're running:", "294 ticks!");
            telemetrize();
        }
        else { //RED_BLUE or weird decision
            // Aiming for the right button (from the front)
            driveForTicks(815, -ALIGN_MIN_POWER);

            telemetry.addData("We're running:", "521 ticks!");
            telemetrize();
        }
        sleep(500);
    }

    public void driveToBeacon() {
        timeMark = runTime.time();

        while ((runTime.time() - timeMark) < 2.5 && opModeIsActive()) {
            setAllDrivePower(.4);
        }

        stopDriveMotors();
    }

    public void op(String state) {
        opState = state;
        telemetrize();
    }


}



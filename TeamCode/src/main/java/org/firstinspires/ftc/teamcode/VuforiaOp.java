import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
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

/**
 * Created by CCA on 11/23/2016.
 */
//@Autonomous(name="VuforiaOp", group="MyGroup")
public class VuforiaOp extends LinearOpMode {
    int leftRedAvg = 0, leftBlueAvg = 0;
    int leftRed = 0, leftBlue = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARVTSuz/////AAAAGRYwaVCcd060r5V5zPNagLKBBcKXizn4VpPEd3T4xIEYhTInQqa9AAn56zmC0i9zNPFczizZ6vg6qZsgFikzbzyDfGkU7xTtM+zdwJ6rrBTVVdHAojTCcwKdFAJ3NFgSBamYM3I2jQqXjE6CxgRJkUDkXa6G2pO2ZOtT2/5JN4s55X9osscF1jeMNkndUrXU4P3dGHbVErHDsTMDd8piqvMLhvCYDLHEKHuvMetrEp/5FJYSDSckMWc6Zffd3rNQYxN0htcDYXf8i6y/yOtyknYiKo4ck5gsgdd/JIQ8X5xhG52bMB9cgVzwD2sJdk1veyY2zBbAxdQNW4PgRlx31jJ0RJZA7ODkHbmcgR2pI1dH";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()) {

            /*To access the image: you need to iterate through the images of the frame object:*/

            VuforiaLocalizer.CloseableFrame frame = locale.getFrameQueue().take(); //takes the frame at the head of the queue
            Image rgb = null;
            Bitmap bm = null;
            int c = 0, red = 0, blue = 0;

            long numImages = frame.getNumImages();


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

            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {

                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

                    /*
                    Vec2F upperLeft = Tool.projectPoint(locale.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
                    Vec2F upperRight = Tool.projectPoint(locale.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
                    Vec2F lowerRight = Tool.projectPoint(locale.getCameraCalibration(), rawPose, new Vec3F(127, -92, 0));
                    Vec2F lowerLeft = Tool.projectPoint(locale.getCameraCalibration(), rawPose, new Vec3F(-127, -92, 0));
                    */


                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(2), translation.get(0)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);

                }
            }

            //average pixel colors of left side

            leftRed = 0;
            leftBlue = 0;

            for (int x = 200; x < 600; x++) {


                for (int y = 360; y < 719; y++) {
                    leftRed += ((bm.getPixel(x, y))&0x00FF0000)>>16;
                    leftBlue += ((bm.getPixel(x, y))&0x000000FF);
                }
            }

            leftRedAvg = leftRed / (400 * 360);
            leftBlueAvg = leftBlue / (400 * 360);

            telemetry.addData("leftRedAvg", leftRedAvg);
            telemetry.addData("leftBlueAvg", leftBlueAvg);


            /*
            telemetry.addData("Color at (0,0): ", bm.getPixel(0, 0));
            telemetry.addData("Color at (1279, 0): ", bm.getPixel(1279, 0));
            telemetry.addData("Color at (0, 719): ", bm.getPixel(0, 719));
            telemetry.addData("Color at (1279, 719): ", bm.getPixel(1279, 719));

            c = bm.getPixel(0,0);
            red = (c&0x00FF0000)>>16;
            blue = (c&0x000000FF);
            telemetry.addData("Red at (0,0): ", red);
            telemetry.addData("Blue at (0,0): ", blue);

            c = bm.getPixel(1279,0);
            red = (c&0x00FF0000)>>16;
            blue = (c&0x000000FF);
            telemetry.addData("Red at (1279,0): ", red);
            telemetry.addData("Blue at (1279,0): ", blue);

            c = bm.getPixel(0,719);
            red = (c&0x00FF0000)>>16;
            blue = (c&0x000000FF);
            telemetry.addData("Red at (0,719): ", red);
            telemetry.addData("Blue at (0,719): ", blue);

            c = bm.getPixel(1279, 719);
            red = (c&0x00FF0000)>>16;
            blue = (c&0x000000FF);
            telemetry.addData("Red at (1279, 719): ", red);
            telemetry.addData("Blue at (1279, 719): ", blue);

            //telemetry.addData("Bitmap Width: ", bm.getWidth());
            //telemetry.addData("Bitmap Height: ", bm.getHeight());

            */
            telemetry.update();
        }

    }
}

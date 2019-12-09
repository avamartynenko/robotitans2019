/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.BlockingQueue;

import static android.content.Context.SENSOR_SERVICE;

/**
 *
 */

//@Autonomous(name="BasicAuton", group="Pushbot")
//@Disabled
public class BasicAuton extends LinearOpMode {
    /* SkyStone tracking variables */
    public static final String TAG = "Vuforia VuMark Sample";
    public static final int SKYSTONE_LEFT = 100;
    public static final int SKYSTONE_CENTER = 200;
    public static final int SKYSTONE_RIGHT = 300;
    public static final int GAME_ALLIANCE_RED = 1000;
    public static final int GAME_ALLIANCE_BLUE = 2000;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final int MOVE_PAUSE = 100; // delay between linear moves
    static final double WALL_OVERRUN = 1; // home much father we run into the wall
    static final double WALL_RECOIL = .5; // how far we pull back from the wall
    static final double START_SPEED = .5; // highest possible speed with no slippage
    static final double MAX_SPEED = 1.0;
    private static final int STONE_HEIGHT = 100;
    // azimuth, pitch and roll
    public float azimuth = 0;
    public float pitch = 0;
    public float roll = 0;
    public int allianceColor = GAME_ALLIANCE_RED;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    ElapsedTime opmodeRunTime = new ElapsedTime();
    SensorManager sManager;
    BlockingQueue que;

    public void setInitVuforia(boolean initVuforia) {
        this.initVuforia = initVuforia;
    }

    public boolean initVuforia = false;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    /* Declare OpMode members. */
    CompetitionHardware robot;
    Arm choiceOfArm;
    // Gravity rotational data
    private float gravity[];

    /*
    public BasicAuton(){
        initialize();
        telemetry.addData("Status Basic Auton", "init complete"+robot.toString());    //
        telemetry.update();
    }
     */
    // Magnetic rotational data
    private float magnetic[]; //for magnetic rotational data
    private float accels[] = new float[3];
    private float mags[] = new float[3];
    private float[] values = new float[3];
    private ElapsedTime runtime = new ElapsedTime();
    private SensorEventListener mySensorEventListener = new SensorEventListener() {
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        public void onSensorChanged(SensorEvent event) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_MAGNETIC_FIELD:
                    mags = event.values.clone();
                    break;
                case Sensor.TYPE_ACCELEROMETER:
                    accels = event.values.clone();
                    break;
            }

            if (mags != null && accels != null) {
                gravity = new float[9];
                magnetic = new float[9];
                SensorManager.getRotationMatrix(gravity, magnetic, accels, mags);
                float[] outGravity = new float[9];
                SensorManager.remapCoordinateSystem(gravity, SensorManager.AXIS_X, SensorManager.AXIS_Z, outGravity);
                SensorManager.getOrientation(outGravity, values);

                azimuth = values[0] * 57.2957795f;
                pitch = values[1] * 57.2957795f;
                roll = values[2] * 57.2957795f;
                mags = null;
                accels = null;
            }
        }
    };

    @Override
    public void runOpMode() {
    }

    public void initialize() {

        telemetry.addData("Status", "initialize BasicAuton" + allianceColor);    //
        telemetry.update();

        switch (allianceColor) {

            case GAME_ALLIANCE_BLUE:
                robot = new ComeptitionHardwareForBlue();
                break;

            default:
                robot = new CompetitionHardware();
                break;

        }

        robot.init(hardwareMap, true, false, true);

        setChoiceOfArm();

        if(initVuforia) initVuforia();
    }

    public int detectSkyStone() {

        return SKYSTONE_CENTER;
    }

    public void goToSkyStone(int skyStonePosition) {

        switch (skyStonePosition) {
            case SKYSTONE_LEFT:

                break;

            case SKYSTONE_CENTER:

                break;

            case SKYSTONE_RIGHT:

                break;

            default:
                break;
        }

    }

    public void setChoiceOfArm() {

        switch (allianceColor) {
            case GAME_ALLIANCE_RED:
                choiceOfArm = robot.frontArm;
                telemetry.addData("ChoiceOfArm", " frontArm");
                telemetry.update();
                break;

            case GAME_ALLIANCE_BLUE:
                choiceOfArm = robot.backArm;
                telemetry.addData("ChoiceOfArm", " backArm");
                telemetry.update();
                break;

            default:
                break;
        }

    }

    public void setChoiceOfArm(Arm armToSet) {
        choiceOfArm = armToSet;
    }

    public void pickUpSkyStone() {

        choiceOfArm.latchStone(0.9);
        choiceOfArm.goDown(0.9);

        sleep(1000);

        //sleep(1000);

        choiceOfArm.liftUp(0.9);
        sleep(1000);

    }

    public void moveToFoundation(int skyStonePosition) {

        switch (skyStonePosition) {
            case SKYSTONE_LEFT:

                break;

            case SKYSTONE_CENTER:

                break;

            case SKYSTONE_RIGHT:

                break;

            default:
                break;
        }

    }

    public void placeSkyStoneOnFoundation() {

        choiceOfArm.goDown(0.75);
        //sleep(3000);

        choiceOfArm.releaseStone(1);
        sleep(500);

        choiceOfArm.liftUp(1);
        sleep(1000);

    }

    public void moveFoundationToBuildZone() {

    }

    public void getCube() {

        telemetry.addData("Status", "Start arm motions...");
        telemetry.update();

        choiceOfArm.goDown(0.5);
        sleep(1500);

        choiceOfArm.stop();
        //sleep(500);

        choiceOfArm.latchStone(0.5);
        sleep(1500);

        choiceOfArm.liftUp(0.5);
        sleep(1500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        choiceOfArm.dropServo.setPower(0);
        //sleep(1000);

        telemetry.addData("Status", "Arm motions complete");
        telemetry.update();
    }

    public void dropCube() {

        telemetry.addData("Status", "Start arm motions...");
        telemetry.update();

        choiceOfArm.goDown(0.5);
        sleep(2500);

        choiceOfArm.stop();
        //sleep(1000);

        choiceOfArm.releaseStone(0.5);
        sleep(2500);

        choiceOfArm.liftUp(0.5);
        sleep(2500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        choiceOfArm.dropServo.setPower(0);
        //sleep(1000);

        telemetry.addData("Status", "Arm motions complete");
        telemetry.update();
    }

    public CompetitionHardware.Direction reverseDirection(CompetitionHardware.Direction Direction) {
        if (Direction == CompetitionHardware.Direction.LEFT)
            return CompetitionHardware.Direction.RIGHT;
        else if (Direction == CompetitionHardware.Direction.FORWARD)
            return CompetitionHardware.Direction.REVERSE;
        else if (Direction == CompetitionHardware.Direction.RIGHT)
            return CompetitionHardware.Direction.LEFT;
        else
            return CompetitionHardware.Direction.FORWARD;
    }

    int linearMoveWrapper(CompetitionHardware.Direction direction, double speed, double distance) {
        return robot.linearMove(direction, speed, distance);
    }

    public void setAllianceColor(int allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void reOrient() {

        if (allianceColor == GAME_ALLIANCE_BLUE) {

            ComeptitionHardwareForBlue.setCurrentOrtientation(ComeptitionHardwareForBlue.ORIENTATION_TWO);

        }
    }

    void saveBitmapToFile(Bitmap bmp) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
        try {
            FileOutputStream outputStream = new FileOutputStream(file);
            try {
                bmp.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
            }
            finally {
                outputStream.close();
                telemetry.log().add("captured %s", file.getName());
            }
        }
        catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
        }
    }

    Bitmap locateStones(Bitmap bmp, CameraCalibration cc, float currPitch, float currRoll) {
        int horizon = horizonLine(bmp, cc, currPitch, currRoll);

        return Bitmap.createBitmap(bmp, horizon - STONE_HEIGHT, 0, 2 * STONE_HEIGHT, bmp.getHeight());
    }

    int horizonLine(Bitmap bmp, CameraCalibration cc, float currPitch, float currRoll) {
        float alpha = cc.getFieldOfViewRads().getData()[0];
        float H = bmp.getWidth();
        float beta = currPitch / 180 * (float) Math.PI;    // pitch, roll and azimuth are stored in degrees in class vars
        float h = H / alpha * beta;

//        telemetry.log().add(String.format("alpha: %.0f, beta: %.0f, h: %d", alpha*180/Math.PI, beta/Math.PI*180, (int)h));

        if (Math.abs(currRoll) < 90)
            h *= -1;
        return bmp.getWidth() / 2 + (int) h;
    }

    public void initVuforia() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AZO77UD/////AAABmbApHmM1UkjohtgDXreA8CWK6va+93R9oaFNMCWiXm4cqFCVkekOfYUCEd2l8wp0z/mP76Aoi56dcwbx5dEvWU5mDkFa/tmYugrCk/uUidZmKLe6nO6z090Bjw8QZA9ZC7HfYc80iIDqLpNHJpKs9ck0fTLT9pKENQ8d//k3iKSG+O/VgYZ3PcdCexlm85uHr5h76sQNkLrzinN9i0ndCNJGH8gp/IKki9OQljWgvCrEvdKKLEvhf5wKxqgjZjjLfpQrYBVl4eRpztAz1CKoMav8lZOaFOjTb0M0kHytfHicFC9eDIEOa15rdEwCn8l1S9O0e6MuvA6yqbg7kohbxNwftRbV+V2M02PMrd927o24";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData(">", "Press Play to start");

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        telemetry.update();
        waitForStart();

        vuforia.enableConvertFrameToBitmap();
        CameraDevice.getInstance().setFlashTorchMode(true);
        vuforia.setFrameQueueCapacity(1);
        que = vuforia.getFrameQueue();

        // Init sensor manager to get camera orientation
        SensorManager sManager = (SensorManager) ((Activity) hardwareMap.appContext).getSystemService(SENSOR_SERVICE);

        sManager.registerListener(mySensorEventListener, sManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
        sManager.registerListener(mySensorEventListener, sManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);
    }

    public int getSkyStonePosition() {
        int iPostion = -1;

        //  sleep(500);
        //  telemetry.addData("Queue capacity: ", que.remainingCapacity());
        //  telemetry.addData("Pitch, azimuth, roll: ", "%.1f %.1f %.1f", pitch, azimuth, roll);
        //  telemetry.update();
        try {
            if (que.remainingCapacity() != 0) {
                float currRoll = roll;
                float currPitch = pitch;
                Bitmap srcBmp = vuforia.convertFrameToBitmap((Frame) que.take());
                if (srcBmp != null) {

                    Bitmap stonesArea = locateStones(srcBmp, vuforia.getCameraCalibration(), currPitch, currRoll);
                    double dstart = opmodeRunTime.milliseconds();
                    int iNumBars = 3;
                    int iLum[] = new int[3];
                    for (int iBar = 0; iBar < iNumBars; iBar++) {
                        int iBarHeight = stonesArea.getHeight() / iNumBars;
                        int iBarStart = iBarHeight * iBar;
                        int iBarEnd = iBarStart + iBarHeight;
                        double r = 0, g = 0, b = 0;
                        int ivStep = 4;
                        int ihStep = 4;
                        for (int x = 0; x < stonesArea.getWidth(); x = x + ihStep) {
                            for (int y = iBarHeight * iBar; y < iBarHeight * (iBar + 1); y = y + ivStep) {
                                int color = stonesArea.getPixel(x, y);
                                r += Color.red(color);
                                g += Color.green(color);
                                b += Color.blue(color);
                            }
                        }
                        r /= stonesArea.getWidth() * iBarHeight / (ivStep * ihStep);
                        g /= stonesArea.getWidth() * iBarHeight / (ivStep * ihStep);
                        b /= stonesArea.getWidth() * iBarHeight / (ivStep * ihStep);

                        iLum[iBar] = (int) (r + g);

//                        telemetry.addData("Col RGB & RG Lum " + iBar, "%d %d %d=%d", (int) r, (int) g, (int) b, iLum[iBar]);
                    }
                    telemetry.addData("Calc Time: ", "%d milliseconds", (int) (opmodeRunTime.milliseconds() - dstart));
                    String sConfig = "000";
                    if (iLum[0] <= iLum[1] && iLum[0] <= iLum[2]) {
                        sConfig = "X00";
                        iPostion = 0;
                    } else if (iLum[1] <= iLum[0] && iLum[1] <= iLum[2]) {
                        iPostion = 1;
                        sConfig = "0X0";
                    } else {
                        iPostion = 2;
                        sConfig = "00X";
                    }
                    telemetry.addData("Configuration: ", "%s", sConfig);
                }
            } else
                telemetry.addData("Unable to get element ", 1);
        }
        catch (InterruptedException ex) {
        }

        return iPostion;
    }

    /**
     * Picks up the stone
     */
    protected void pickupStone() {
        choiceOfArm.latchStone(1.0);
        choiceOfArm.goDown(1.0);
        sleep(2000);
        choiceOfArm.liftUp(1.0);
    }
}
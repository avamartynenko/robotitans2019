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

package org.firstinspires.ftc.teamcode.util;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CompetitionHardwareEx;

import java.text.DecimalFormat;

import static android.content.Context.SENSOR_SERVICE;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

//import org.firstinspires.ftc.teamcode.util.SensorManagerHelper;

/**
 * {@link DistanceSensors} illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry {@link Telemetry#log() log} is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
@TeleOp(name="Diag: Distance Sensors", group = "Util")
//@Disabled
public class DistanceSensors extends LinearOpMode  {
    /** keeps track of the line of the poem which is to be emitted next */
    int poemLine = 0;

    /** keeps track of how long it's been since we last emitted a line of poetry */
    ElapsedTime poemElapsed = new ElapsedTime();

    SensorManager sManager;
    // Gravity rotational data
    private float gravity[];
    // Magnetic rotational data
    private float magnetic[]; //for magnetic rotational data
    private float accels[] = new float[3];
    private float mags[] = new float[3];
    private float[] values = new float[3];

    // azimuth, pitch and roll
    public float azimuth = 0;
    public float pitch=0;
    public float roll=0;

    CompetitionHardwareEx robot = new CompetitionHardwareEx();


    static final String[] poem = new String[] {

        "Mary had a little lamb,",
        "His fleece was white as snow,",
        "And everywhere that Mary went,",
        "The lamb was sure to go.",
        "",
        "He followed her to school one day,",
        "Which was against the rule,",
        "It made the children laugh and play",
        "To see a lamb at school.",
        "",
        "And so the teacher turned it out,",
        "But still it lingered near,",
        "And waited patiently about,",
        "Till Mary did appear.",
        "",
        "\"Why does the lamb love Mary so?\"",
        "The eager children cry.",
        "\"Why, Mary loves the lamb, you know,\"",
        "The teacher did reply.",
        "",
        ""
    };



    private static final int REQUEST_CAPTURE_IMAGE = 100;

    @Override public void runOpMode() {

        robot.init(hardwareMap, false, false, true);


        /* we keep track of how long it's been since the OpMode was started, just
         * to have some interesting data to show */
        ElapsedTime opmodeRunTime = new ElapsedTime();

        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(6);
        // The interval between lines of poetry, in seconds
        double sPoemInterval = 0.6;

        /**
         * Wait until we've been given the ok to go. For something to do, we emit the
         * elapsed time as we sit here and wait. If we didn't want to do anything while
         * we waited, we would just call {@link #waitForStart()}.
         */
        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        // Ok, we've been given the ok to go

        SensorManager sManager = (SensorManager) ((Activity) hardwareMap.appContext).getSystemService(SENSOR_SERVICE);

        sManager.registerListener(mySensorEventListener, sManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
        sManager.registerListener(mySensorEventListener, sManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);


        /**
         * As an illustration, the first line on our telemetry display will display the battery voltage.
         * The idea here is that it's expensive to compute the voltage (at least for purposes of illustration)
         * so you don't want to do it unless the data is <em>actually</em> going to make it to the
         * driver station (recall that telemetry transmission is throttled to reduce bandwidth use.
         * Note that getBatteryVoltage() below returns 'Infinity' if there's no voltage sensor attached.
         *
         * @see Telemetry#getMsTransmissionInterval()
         */
        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
            });

        // Reset to keep some timing stats for the post-'start' part of the opmode
        opmodeRunTime.reset();
        int loopCount = 1;

        DecimalFormat df = new DecimalFormat("#.#");



        // Go go gadget robot!
        while (opModeIsActive()) {

//            smh = new SensorManagerHelper(hardwareMap);
            //telemetry.addData("Azimuth, pitch, roll: ", "%.3f %.3f %.3f", azimuth, pitch, roll);
            //telemetry.addData("Pitch: ", "%.3f", pitch);
            //telemetry.addData("Angles: ", "%.3f %.3 %.3f", .1, .2, .3);

            // Emit poetry if it's been a while
   /*         if (poemElapsed.seconds() > sPoemInterval) {
                emitPoemLine();
            }*/

            // As an illustration, show some loop timing information
            telemetry.addData("loop count", loopCount);
            telemetry.addData("ms/loop", "%.3f ms", opmodeRunTime.milliseconds() / loopCount);

            // Show joystick information as some other illustrative data
            telemetry.addLine("Distance Sensors")
                    .addData("Front", df.format(robot.sensorTimeOfFlightF.getDistance(INCH)))
                    .addData("Back", df.format(robot.sensorTimeOfFlightB.getDistance(INCH)))
                    .addData("Left", df.format(robot.sensorTimeOfFlightL.getDistance(INCH)))
                    .addData("Right", df.format(robot.sensorTimeOfFlightR.getDistance(INCH)));

            /**
             * Transmit the telemetry to the driver station, subject to throttling.
             * @see Telemetry#getMsTransmissionInterval()
             */
            telemetry.update();

            /** Update loop info and play nice with the rest of the {@link Thread}s in the system */
            loopCount++;
        }
    }

    // emits a line of poetry to the telemetry log
    void emitPoemLine() {
        telemetry.log().add(poem[poemLine]);
        poemLine = (poemLine+1) % poem.length;
        poemElapsed.reset();
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

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
                SensorManager.remapCoordinateSystem(gravity, SensorManager.AXIS_X,SensorManager.AXIS_Z, outGravity);
                SensorManager.getOrientation(outGravity, values);

                azimuth = values[0] * 57.2957795f;
                pitch =values[1] * 57.2957795f;
                roll = values[2] * 57.2957795f;
                mags = null;
                accels = null;
            }
        }
    };

}

/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.POC;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorREV2mDistanceWithMove} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */

@TeleOp(name = "POC: REV2mDistance", group = "POC")
@Disabled
public class SensorREV2mDistanceWithMove extends LinearOpMode {
    private DistanceSensor sensorRange;
    double avgDistance = 0;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "distance_front");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        Thread  readerThread = new MeterReader(sensorTimeOfFlight);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
    //    readerThread.start();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            linearMoveToDistance(sensorTimeOfFlight, 10);
        }

        readerThread.interrupt();
    }

    private class MeterReader extends Thread
    {
        Rev2mDistanceSensor sensorTimeOfFlight;
        Queue<Double> q = new LinkedList<>();

        public MeterReader(Rev2mDistanceSensor sensor)
        {
            this.setName("DriveThread");
            sensorTimeOfFlight = sensor;

            telemetry.addData("%s", this.getName());
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            telemetry.addData("Starting thread %s",this.getName());

            try
            {
                int counter = 0;
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    sleep(10);
                    while(q.size() > 10)
                        q.remove();

                    q.add(sensorRange.getDistance(DistanceUnit.INCH));
                    avgDistance = 0;
                    for(double dist : q)
                        avgDistance += dist;
                    avgDistance /= q.size();
//                    telemetry.addData(this.getName(), counter++);

/*                    telemetry.addData("deviceName",sensorRange.getDeviceName() );
                    telemetry.addData("range", String.format("%.01f mm avg", avgDistance));
                    telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
                    telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
                    telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
                    telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

                    // Rev2mDistanceSensor specific methods.
                    telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                    telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                    telemetry.update();*/

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
//            catch (InterruptedException e) {telemetry.addData("%s interrupted", this.getName());}
            // an error occurred in the run loop.
            catch (Exception e) {telemetry.addData("Exception:", e.toString());}

            telemetry.addData("end of thread %s", this.getName());
        }
    }

    public void linearMoveToDistance(Rev2mDistanceSensor sensor, double distnaceToTravel)
    {
        ElapsedTime et = new ElapsedTime();
        double startSpeed = .3;
        double maxSpeed = 1;
        double minSpeed = .2;
        double accellerationTime = 3000;
        double slowDownDistance = 2;
        double slowSteps = 4;
        int direction = (distnaceToTravel > sensor.getDistance(DistanceUnit.INCH)) ? -1 : 1;
        Queue<Double> q = new LinkedList<>();
        double precision = .5;

        ElapsedTime loopTimer = new ElapsedTime();

//        double slowDownStep = 1;
        double currDistance = sensor.getDistance(DistanceUnit.INCH);
        while(Math.abs(currDistance - distnaceToTravel) > slowDownDistance)
        {
            while(q.size() > 10)
                q.remove();

            q.add(currDistance);
            avgDistance = 0;
            for(double dist : q)
                avgDistance += dist;
            avgDistance /= q.size();
            direction = (distnaceToTravel > currDistance) ? -1 : 1;

            telemetry.addLine("Accelerating and moving at full speed");
            double currentSpeed = Math.min(accellerationTime, et.milliseconds())/accellerationTime*maxSpeed;
            currentSpeed *= direction;
            telemetry.addData("Current speed, distance, lag, loop time(ms)", "%.1f %.1f %.1f %.0f", currentSpeed, avgDistance, currDistance - avgDistance, loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
            currDistance = sensor.getDistance(DistanceUnit.INCH);
        }

        while (Math.abs(currDistance - distnaceToTravel) > precision)
        {
            telemetry.addLine("Decelerating to full stop");

            while(q.size() > 10)
                q.remove();

            q.add(currDistance);
            avgDistance = 0;
            for(double dist : q)
                avgDistance += dist;
            avgDistance /= q.size();
            direction = (distnaceToTravel > currDistance) ? -1 : 1;

            //double currentSpeed = minSpeed + (maxSpeed-minSpeed)*Math.abs(currDistance - distnaceToTravel)/slowDownDistance;
            //double currentSpeed = minSpeed + (maxSpeed-minSpeed)/slowSteps * Math.floor(currDistance*slowSteps/slowDownDistance);
            double currentSpeed = minSpeed + Math.abs(currDistance - distnaceToTravel)/slowDownDistance*(maxSpeed-minSpeed);
            currentSpeed *= direction;

            telemetry.addData("Current speed, distance, lag, loop time(ms)", "%.1f %.1f %.1f %.0f", currentSpeed, avgDistance, currDistance - avgDistance, loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
            currDistance = sensor.getDistance(DistanceUnit.INCH);
        }

        telemetry.addLine("Path completed");
        telemetry.update();
        sleep(1000);
    }
}
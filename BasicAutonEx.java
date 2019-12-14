package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.RIGHT;

public class BasicAutonEx extends BasicAuton {
    public CompetitionHardwareEx robot;

    // all below dimensions are in Inches
    protected static final double FRONT_ARM_OFFSET = 5; //distance from from ARM to Front sensor
    protected static final double STONE_LENGTH = 8; // lenght of skystone
    protected double FIRST_STONE_DROP = 105;
    protected static final double FIELD_WIDTH = 46+46+45;
    protected static final double ROBOT_LENGTH = 17;
    protected static final double ROBOT_WIDTH = 17.5;
    protected static final double DROP_POSITION = 27.5;

    @Override
    public void initialize() {
        telemetry.addData("Status", "Initialize BasicAutonEx " + allianceColor);
        telemetry.update();

        robot = new CompetitionHardwareEx();
        robot.init(hardwareMap, true, false, true);
        super.robot = robot;

        setChoiceOfArm();

        initVuforia();
    }

    public void pickUpSkyStone(boolean delay) {

        choiceOfArm.latchStone(0.9);
        choiceOfArm.goDown(0.9);

        sleep(1000);
        if(delay)
            sleep(500);

        //sleep(1000);

        choiceOfArm.liftUp(0.9);
        sleep(1000);

    }

    protected double getSensorValue(Rev2mDistanceSensor sensor, DistanceUnit UOM) {
        double dResult;
        boolean bTimeOut;
        int iReadAttempt = 0;

        do {
            iReadAttempt++;
            dResult = sensor.getDistance(UOM);
            bTimeOut = sensor.didTimeoutOccur();
            if(bTimeOut) {
                telemetry.log().add("Sensor " + sensor.getDeviceName() + " read timeout. Retry # " + iReadAttempt + " Sensor read value " + dResult);
                telemetry.update();
                sleep(10 * iReadAttempt);
            }
        }
        while(bTimeOut && iReadAttempt <3); // allow three attempts to read the sensor

        return dResult;
    }

    public void correctGain(boolean bFoundation) {
        // correct gain
        double distanceFromLWall = getSensorValue(robot.sensorTimeOfFlightL, INCH);
        telemetry.log().add("Distance to Left Wall (in) " + String.format("%.1f", distanceFromLWall));
        telemetry.update();

        if (distanceFromLWall < 27) {
            robot.linearMove(RIGHT, .6, 27 - distanceFromLWall - 1 - (bFoundation ? .5 : .5), this);
            telemetry.log().add("Correction Right (in) " + String.format("%.1f", 27 - distanceFromLWall));
        }

        if (distanceFromLWall > 28) {
            robot.linearMove(LEFT, .6, distanceFromLWall - 28 + .5 - (bFoundation ? .5 : 0), this);
            telemetry.log().add("Correction Left (in) " + String.format("%.1f", distanceFromLWall - 28.5 + .5));
        }

        telemetry.update();
    }
}

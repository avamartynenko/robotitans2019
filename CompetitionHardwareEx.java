package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.LynxOptimizedI2cFactory;

import java.util.Iterator;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.GYRO_LEFT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.GYRO_RIGHT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.RIGHT;

public class CompetitionHardwareEx extends CompetitionHardware {
    public double opStartHeading = 0;
    public boolean verboseTelemetry = false;

    protected DistanceSensor sensorRangeF;
    protected DistanceSensor sensorRangeB;
    protected DistanceSensor sensorRangeL;
    protected DistanceSensor sensorRangeR;

    // you can also cast this to a Rev2mDistanceSensor if you want to use added
    // methods associated with the Rev2mDistanceSensor class.
    public Rev2mDistanceSensor sensorTimeOfFlightL;
//    public Rev2mDistanceSensor sensorTimeOfFlightR;
    public Rev2mDistanceSensor sensorTimeOfFlightF;
    public Rev2mDistanceSensor sensorTimeOfFlightB;

/*    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor frontLeft;

    protected ExpansionHubEx revMaster;
    protected ExpansionHubEx revSlave;
    protected RevBulkData revExpansionMasterBulkData;
    protected RevBulkData revExpansionSlaveBulkData;

    private ArrayList<RevMotor> allMotors = new ArrayList<>();*/

    protected ElapsedTime etTimer;
    protected long lastUpdateMasterTime = 0;

    final double GLOBAL_MAX_SPEED = 1;

    protected boolean I2CWindowsReset = false;
    protected boolean USE_FAST_I2C = false;

    @Override
    public void init(HardwareMap hMap, boolean needEncoder, boolean needColorSensor, boolean needGyro) {
        super.init(hMap, needEncoder, needColorSensor, true);

/*        revMaster = hwMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        revSlave = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        // TODO: disable for prod config
        revMaster.setPhoneChargeEnabled(true);*/

        sensorRangeF = hMap.get(DistanceSensor.class, "distance_front");
        sensorRangeB = hMap.get(DistanceSensor.class, "distance_back");
        sensorRangeL = hMap.get(DistanceSensor.class, "distance_left");
//        sensorRangeR = hMap.get(DistanceSensor.class, "distance_right");

        if(USE_FAST_I2C) {
            sensorRangeF.close();
            sensorRangeB.close();
            sensorRangeL.close();
//            sensorRangeR.close();

            Iterator<LynxModule> lmIterator = hwMap.getAll(LynxModule .class).iterator();
            LynxModule module = null;

            while(lmIterator.hasNext()) {
                LynxModule currentModule = lmIterator.next();
                module = currentModule;
                if(currentModule.isParent()) {
                    sensorTimeOfFlightL = new Rev2mDistanceSensor(LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(module, 1));
//                    sensorTimeOfFlightR = new Rev2mDistanceSensor(LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(module, 2));
                    sensorTimeOfFlightF = new Rev2mDistanceSensor(LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(module, 3));
                }
                else {
                    sensorTimeOfFlightB = new Rev2mDistanceSensor(LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(module, 2));
                }
            }
     }
        else {
            sensorTimeOfFlightL = (Rev2mDistanceSensor) sensorRangeL;
//            sensorTimeOfFlightR = (Rev2mDistanceSensor) sensorRangeR;
            sensorTimeOfFlightF = (Rev2mDistanceSensor) sensorRangeF;
            sensorTimeOfFlightB = (Rev2mDistanceSensor) sensorRangeB;
        }



/*        backLeft = (ExpansionHubMotor) super.backLeft;
        backRight = (ExpansionHubMotor) super.backRight;
        frontRight = (ExpansionHubMotor) super.frontRight;
        frontLeft = (ExpansionHubMotor) super.frontLeft;

        //add them to all motors
        RevMotor fl = new RevMotor((ExpansionHubMotor) hwMap.get("frontLeft"),true);
        RevMotor fr = new RevMotor((ExpansionHubMotor) hwMap.get("frontRight"),true);
        RevMotor bl = new RevMotor((ExpansionHubMotor) hwMap.get("backLeft"),true);
        RevMotor br = new RevMotor((ExpansionHubMotor) hwMap.get("backRight"),true);

        allMotors.add(fl);
        allMotors.add(fr);
        allMotors.add(bl);
        allMotors.add(br);*/

        etTimer = new ElapsedTime();
    }

    // sets robot's heading relative to its starting heading,
    // eg 0 returns root to its initial orientation, 90, turns to 90 relative to start orientation, et
    public void setHeading(double newHeading, LinearOpMode opMode) {
        double turnAngle = calcTurnAngleDEx(getActualHeading(), opStartHeading + newHeading);
        opMode.telemetry.log().add("setHeading ActualHeading/opStartHeading/turnAngle: " + getActualHeading() +"/" + opStartHeading + "/" + turnAngle);
        if(turnAngle != 0) {
            Direction correctionAngleDirection = (turnAngle > 0) ? GYRO_LEFT : GYRO_RIGHT;
            gyroMoveByOffset(correctionAngleDirection, GLOBAL_MAX_SPEED, Math.abs(turnAngle), opMode);
        }
    }

    /**
     * Calculate angle values between two directions in degrees
     * @param startAngle the initial heading
     * @param currentAngle current heading
     * @return different between initial and current heading
     */
    protected double calcTurnAngleDEx(double startAngle, double currentAngle) {
        double turnAngle = currentAngle - startAngle;
        turnAngle = (turnAngle > 360) ? turnAngle - 360 : turnAngle;
        turnAngle = (turnAngle < -360) ? turnAngle + 360 : turnAngle;

        turnAngle = (turnAngle > 180) ? 360 - turnAngle : turnAngle;
        turnAngle = (turnAngle < -180) ? 360 + turnAngle : turnAngle;

        return turnAngle;
    }

    public int linearMoveEncoder(Direction direction, double speed, double distance, @Nullable LinearOpMode opMode) {
        int positionTarget = (int)(distance * COUNTS_PER_INCH);

        // encoder position does not get reset between program starts
        // motors can start in random positions if robot was not restarted after completion of previous program
        setMotorsMode(STOP_AND_RESET_ENCODER);
        setMotorsMode(RUN_TO_POSITION);
        setZeroPowerMode(BRAKE);

        setDirection(direction);

        // run w/o encoder since we are controlling motor speed
        // Determine new target position, and pass to motor controller
        backLeft.setTargetPosition(positionTarget);
        backRight.setTargetPosition(positionTarget);
        frontLeft.setTargetPosition(positionTarget);
        frontRight.setTargetPosition(positionTarget);

        if(opMode != null && verboseTelemetry) {
            opMode.telemetry.log().add("Linear move direction: " + direction2string(direction) + ". Distance: " + distance + " inches");
            opMode.telemetry.log().add("Speed profile " + (activateSpeedProfile ? "ON" : "OFF"));
        }

        super.setPower4WDrive(speed);

        return positionTarget;
    }

    @Override
    public int linearMove(Direction direction, double speed, double distance, @Nullable LinearOpMode opMode) {

        int positionTarget = (int)(distance * COUNTS_PER_INCH);

        // encoder position does not get reset between program starts
        // motors can start in random positions if robot was not restarted after completion of previous program
        setMotorsMode(STOP_AND_RESET_ENCODER);
        setMotorsMode(RUN_WITHOUT_ENCODER);
        setZeroPowerMode(BRAKE);

        setDirection(direction);
        //setMotorsMode(RUN_TO_POSITION);
        //setMotorsMode(RUN_USING_ENCODER);

        // run w/o encoder since we are controlling motor speed
        // Determine new target position, and pass to motor controller
        backLeft.setTargetPosition(positionTarget);
        backRight.setTargetPosition(positionTarget);
        frontLeft.setTargetPosition(positionTarget);
        frontRight.setTargetPosition(positionTarget);

        if(opMode != null && verboseTelemetry) {
            opMode.telemetry.log().add("Linear move direction: " + direction2string(direction) + ". Distance: " + distance + " inches");
            opMode.telemetry.log().add("Speed profile " + (activateSpeedProfile ? "ON" : "OFF"));
        }


        set4WDriveWithSpeedProfile(direction, speed, positionTarget, opMode);

        return positionTarget;
    }

    public void set4WDriveWithSpeedProfile(Direction direction, double speed, int target, LinearOpMode opMode) {
        double accelerationTime = 0;
        double breakDistance = 0;
        double currentSpeed = 0;
        double minBreakSpeed = 0;
        double correctedTarget = target;
        double posIncrement = 0;

        DcMotor motor = backLeft;

        if(opMode != null && verboseTelemetry)
            opMode.telemetry.log().add("Entering CompetitionHWEx.set4WDriveWithSpeedProfile");

        double initialHeading = getActualHeading();

        // set min break speed, acceleration time and break distance
        switch (direction) {
            case FORWARD:
            case REVERSE:
                minBreakSpeed = .25;
                breakDistance = 450;
                accelerationTime = 350;
                break;
            case LEFT:
            case RIGHT:
                minBreakSpeed = .25;
                breakDistance = 350;
                accelerationTime = 500;
                break;
        }

        // strafe adjustments
        switch (direction) {
            case LEFT:
                posIncrement = target*.098;
                motor = backLeft;
                break;
            case RIGHT:
                posIncrement = target*.115;
                motor = backRight;
                break;
            default:
                if(opMode != null && verboseTelemetry)
                    opMode.telemetry.log().add("Target correction not required");
                break;
        }

        correctedTarget += posIncrement;

        // adjust target position based on motors characteristics
        backLeft.setTargetPosition(backLeft.getTargetPosition() + (int)posIncrement);
        backRight.setTargetPosition(backRight.getTargetPosition() + (int)posIncrement);
        frontLeft.setTargetPosition(frontLeft.getTargetPosition() + (int)posIncrement);
        frontRight.setTargetPosition(frontRight.getTargetPosition() + (int)posIncrement);

        resetAngle();
        double kp = .05;
        if(direction == FORWARD)
            kp = .15;

        PIDController pidDrive = new PIDController(kp, 0, 0);
        // Set up parameters for driving in a straight line.

        pidDrive.setSetpoint(getAngle());
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        double correction = 0;

        if(opMode != null && verboseTelemetry) {
            opMode.telemetry.log().add("Current Positions bl " + backLeft.getCurrentPosition() + " br " + backRight.getCurrentPosition() + " fl " + frontLeft.getCurrentPosition() + " fr " + frontRight.getCurrentPosition());
            opMode.telemetry.log().add("Target Positions bl " + backLeft.getTargetPosition() + " br " + backRight.getTargetPosition() + " fl " + frontLeft.getTargetPosition() + " fr " + frontRight.getTargetPosition());
            opMode.telemetry.log().add("Heading " + getActualHeading() + ". Angle: " + getAngle());
            opMode.telemetry.log().add("Target original/corrected/posIncrement " + target + "/" + correctedTarget + "/" + posIncrement);
        }

        int currentPosition = motor.getCurrentPosition();
        ElapsedTime et = new ElapsedTime();
        while (currentPosition <= Math.max(correctedTarget - breakDistance, correctedTarget/2) && opMode.opModeIsActive()) {
            // Use PID with imu input to drive in a straight line.


            currentPosition = motor.getCurrentPosition();
            currentSpeed = Range.clip(Math.min(accelerationTime, et.milliseconds()) / accelerationTime, .20, Math.min(GLOBAL_MAX_SPEED, speed));

            pidDrive.setOutputRange(0, Math.min(currentSpeed, GLOBAL_MAX_SPEED));
            correction = pidDrive.performPID(getAngle());

            if(direction == REVERSE || direction == RIGHT)
                correction *= -1;

            setPower4WDrive(  Range.clip(currentSpeed - correction, -1, 1),
                    Range.clip(currentSpeed + correction, -1, 1),
                    Range.clip(currentSpeed - correction, -1, 1),
                    Range.clip(currentSpeed + correction, -1, 1));
        }

        if(opMode != null && verboseTelemetry) {
            opMode.telemetry.log().add("Final correction " + correction);
            opMode.telemetry.log().add("Pre break heading " + getActualHeading());
            opMode.telemetry.log().add("Prebreak position bl " + backLeft.getCurrentPosition() + " br " + backRight.getCurrentPosition() + " fl " + frontLeft.getCurrentPosition() + " fr " + frontRight.getCurrentPosition());
        }

        double maxTravelSpeed = currentSpeed; // capture current speed

        // stop braking as soon as at least one motor will reach target position
        while (!oneMotorAtTarget() && opMode.opModeIsActive()) {
            double slowDownCorrection;
            // Use PID with imu input to drive in a straight line.
            currentSpeed = Math.max((correctedTarget-currentPosition)/breakDistance, minBreakSpeed);
            pidDrive.setOutputRange(0, Math.min(currentSpeed, GLOBAL_MAX_SPEED));

            slowDownCorrection = pidDrive.performPID(getAngle());

            currentPosition = motor.getCurrentPosition();
            if(direction == REVERSE || direction == RIGHT)
                slowDownCorrection *= -1;

            setPower4WDrive(  Range.clip(currentSpeed - slowDownCorrection, -1, 1),
                    Range.clip(currentSpeed + slowDownCorrection, -1, 1),
                    Range.clip(currentSpeed - slowDownCorrection, -1, 1),
                    Range.clip(currentSpeed + slowDownCorrection, -1, 1));
        }

        // stop
        setPower4WDrive(0);
        waitToStop();

        double correctionAngle = calcTurnAngleD(initialHeading, getAbsoluteHeading());

        if (opMode != null && verboseTelemetry) {
            opMode.telemetry.log().add("Positions bl " + backLeft.getCurrentPosition() + " br " + backRight.getCurrentPosition() + " fl " + frontLeft.getCurrentPosition() + " fr " + frontRight.getCurrentPosition());
            opMode.telemetry.log().add("Final heading/correction angle " + getActualHeading() + "/" + correctionAngle);
        }

        correctHeading(initialHeading, opMode);
        setPower4WDrive(0);


        //setEncoder(true);
    }

    @Override
    public void correctHeading(double initialHeading, LinearOpMode opMode) {
        double correctionAngle = calcTurnAngleDEx(initialHeading, getActualHeading());

        if(opMode != null && verboseTelemetry) {
            opMode.telemetry.log().add("correctHeading correction Angle: " + correctionAngle);
            opMode.telemetry.update();
        }

        if(correctionAngle != 0) {
            Direction correctionAngleDirection = (correctionAngle < 0) ? GYRO_LEFT : GYRO_RIGHT;
            gyroMoveByOffset(correctionAngleDirection, GLOBAL_MAX_SPEED, Math.abs(correctionAngle), opMode);
        }
    }

    @Override
    public void enableGyro () {
        if(!gyroInitialized || !I2CWindowsReset) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = getREVName() + "_AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = false;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            if(imu != null) {
                imu.close();
            }

            Iterator<LynxModule> lmIterator = hwMap.getAll(LynxModule .class).iterator();
            LynxModule module = null;

            while(lmIterator.hasNext()) {
                LynxModule currentModule = lmIterator.next();
                if(currentModule.isParent()) {
                    module = currentModule;
                    break;
                }
            }

            imu = new LynxEmbeddedIMU(LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(module, 0));
            imu.initialize(parameters);

            while(!imu.isGyroCalibrated()){
                try {
                    sleep(50);
                }
                catch (Exception e) {
                    Log.e("IMU", "Calibration terminated");
                }
            }

            gyroInitialized = true;
            I2CWindowsReset = true;
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        //imu.stopAccelerationIntegration();
    }

    /*
    protected int minMotorPosition() {
        RevBulkData newDataMaster;

        try {
            newDataMaster = revMaster.getBulkInputData();
            if(newDataMaster != null){
                revExpansionMasterBulkData = newDataMaster;
            }
        }
        catch(Exception e) {
            //don't set anything if we get an exception
        }
        lastUpdateMasterTime = etTimer.nanoseconds();

        /////NOW WE HAVE THE BULK DATA BUT WE NEED TO SET THE MOTOR POSITIONS/////
        for(RevMotor revMotor : allMotors){
            if(revMotor == null){continue;}
            if(revMotor.isMaster){
                if(revExpansionMasterBulkData != null){
                    revMotor.setEncoderReading(
                            revExpansionMasterBulkData.getMotorCurrentPosition(revMotor.myMotor));
                }
            }else{
                if(revExpansionSlaveBulkData != null){
                    revMotor.setEncoderReading(
                            revExpansionSlaveBulkData.getMotorCurrentPosition(revMotor.myMotor));
                }
            }
        }

        int iResult = allMotors.get(0).getCurrentPosition();
        for(RevMotor revMotor : allMotors) {
            iResult = Math.min(iResult, revMotor.getCurrentPosition());
        }

        return iResult;
    }

     */
}

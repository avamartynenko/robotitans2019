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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import static java.lang.Thread.sleep;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class CompetitionHardware
{
    /* Public OpMode members. */
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public ColorSensor colorsense = null;
    public boolean hasHook = true;
    public boolean hasArm = true;
    public boolean activateSpeedProfile = false;
    public Hooks hookLatch = null;
    //public ServoHooks hookLatch = null;
    public IntakeMech intakeMech = null;
    public LiftMech liftMech = null;
    public Arm frontArm = null;
    public Arm backArm = null;

    public static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.66 ;     // This is < 1.0 if geared UP
    public final double     COUNTS_PER_MOTOR_REV_WITH_GEAR    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    public final double     WHEEL_CIRCUMFRENCE      = WHEEL_DIAMETER_INCHES * 3.1415;

    public final double  MECHANUM_CPI_RATIO  = 0.5; //since it is a 4 wheel drive, you have to half the CPI

    public  final double   COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV_WITH_GEAR/ WHEEL_CIRCUMFRENCE) * MECHANUM_CPI_RATIO;

    public enum Direction {
        FORWARD,
        REVERSE,
        RIGHT,
        LEFT,
        GYRO_RIGHT,
        GYRO_LEFT,
        DIAGONAL_RIGHT_UP,
        DIAGONAL_RIGHT_DOWN,
        DIAGONAL_LEFT_UP,
        DIAGONAL_LEFT_DOWN
    }

    public final double MAX_SPEED = 1;          // defines max robot speed. set to 1 for normal operation, .5 for debugging to observer robot moves in slow motion
    public final double TURN_ERROR = 43;        // Robot over rotates by approximaterly 45 degrees if directed to turn 90 degrees at full speed
    public final double NO_ERROR_SPEED = .14;   // max speed at which robot turns correctly, robot may not move any any speed less than .14
    public final double STOP_GAP = 3;           // even at smallest possible speed there is still lag in processing wich results in overrun in case if power is killed once the angle is reach
                                                // we need to kill power 3 degrees ahead of target angle to achive accurate turn angle
                                                // STPO_GAP is dependent on turn speed and robot configuration and will need to be reset in case if robot configuraiton is changed
    public final long OVERRAN_SLEEP_TIME = 50;  // how long robot should sleep for in uncontrolled spin before checking if it is stabilized

    private boolean gyroInitialized = false;

    // variables for using gryo
    Orientation angles;
    public BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CompetitionHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hMap, boolean needEncoder,boolean needColorSensor, boolean needGyro) {
        this.hwMap=hMap;
        // Define and Initialize Motors
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");

        if (hasHook) hookLatch = new Hooks(hwMap);
        if (hasArm) frontArm = new Arm(hwMap,Arm.FRONT_ARM);
        if (hasArm) backArm = new Arm(hwMap, Arm.BACK_ARM);

        //Sets direction of motors
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        setPower4WDrive(0);

        setEncoder(needEncoder);

        if(needColorSensor)
            enableColorSensor();

        if(needGyro)
            enableGyro();

        /// Keep the attachments to home position
        hookLatch.release();
       // frontArm.moveToHomePosition();
       // backArm.moveToHomePosition();

    }

    public void initTeleopModules() {
        intakeMech = new IntakeMech(hwMap);
        liftMech = new LiftMech(hwMap);
    }

    public void setEncoder(boolean needEncoder){

        if (!needEncoder) {
            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        else {

            /// Stop and Reset ENCODER
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //RUN USING ENCODER
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void enableGyro (){

        if(!gyroInitialized) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            while(!imu.isGyroCalibrated()){
                try {
                    sleep(50);
                }
                catch (Exception e) {
                    Log.i("IMU", "Calibration terminated");
                }
            }

            gyroInitialized = true;
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        //imu.stopAccelerationIntegration();
    }

    public void enableColorSensor(){
        // Gets Sensors
        colorsense = hwMap.get(ColorSensor.class, "color");
        colorsense.enableLed(true);
    }

    public int linearMove (Direction direction,double speed, double distance) {
        int newBleftTarget;
        int newBrightTarget;
        int newFleftTarget;
        int newFrightTarget;

        // Determine new target position, and pass to motor controller
        newBleftTarget = backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newBrightTarget = backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newFleftTarget = frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newFrightTarget = frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        setDirection(direction);

        backLeft.setTargetPosition(newBleftTarget);
        backRight.setTargetPosition(newBrightTarget);
        frontLeft.setTargetPosition(newFleftTarget);
        frontRight.setTargetPosition(newFrightTarget);

        // Turn On RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!activateSpeedProfile) {
            //Setting the power
            setPower4WDrive(Math.abs(speed));

            while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
                // just waiting when motors are busy
            }

        } else {
            set4WDriveWithSpeedProfile(Math.abs(speed), newBleftTarget, backLeft);

        }

        // Stop all motion
        setPower4WDrive(0);
        setEncoder(true);

        return newBleftTarget;
    }

    public void set4WDriveWithSpeedProfile(double speed, int target, DcMotor motor){
        double profileSpeed = 0.0;
        double speedIncrement = 0.007;
        int curPostion = motor.getCurrentPosition();

        while(curPostion <= target){

            if(curPostion < target/2){
                profileSpeed = profileSpeed + speedIncrement;
            } else{
                profileSpeed = profileSpeed - speedIncrement;
            }

            if(profileSpeed > speed) profileSpeed = speed;
            if(profileSpeed < 0.0) profileSpeed = 0.0;

            setPower4WDrive(profileSpeed);
            curPostion = motor.getCurrentPosition();
        }

        setPower4WDrive(0);
        setEncoder(true);
    }

    public void setPower4WDrive(double speed){
        setPower4WDrive(speed,speed,speed,speed);
    }

    // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
    public void setPower4WDrive(double BleftDriveSpeed, double BrightDriveSpeed, double FleftDriveSpeed, double FrightDriveSpeed ){
        backLeft.setPower(BleftDriveSpeed);
        backRight.setPower(BrightDriveSpeed);
        frontLeft.setPower(FleftDriveSpeed);
        frontRight.setPower(FrightDriveSpeed);
    }

    /*
     we are setting what needs to be done(forward,backward,right,left)
     when we put in each thing(forward,backward,right,left) it will come here and look for that
     */
    public void setDirection(Direction direction){

        switch (direction){
            case FORWARD:
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case REVERSE:
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;

            case RIGHT:
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case LEFT:
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE); // INSTEAD OF REVERSE WE WENT FORWARD
                break;
            case GYRO_LEFT:
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case GYRO_RIGHT:
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;

            default:
                break;
        }
    }

    // WARNING: due to processing lag and robot inertia actual angle will usaully be different
    // from requested angle except for very small speeds
    // eg 90 degrees turn exected at full speed will result in approximately 45 degrees overrun
    // use gyroMove2 in case if precision is required
    public void gyroMove(Direction direction,double speed, double angle) {

        enableGyro();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        setDirection(direction);

        //Setting the power
        while(getAbsoluteHeading() < angle) {
            setPower4WDrive(Math.abs(speed));
        }
        setPower4WDrive(0.0);

        imu.stopAccelerationIntegration();
    }

    // DO NOT USE TO ROTATE THE PLATFORM!!!
    public void gyroMove90(Direction direction, Telemetry telemetry) {
        gyroMove2(direction, 90, telemetry);
    }

    // DO NOT USE TO ROTATE THE PLATFORM!!!
    public void gyroMove2(Direction direction, double angle, double speed, Telemetry telemetry) {
        double startAngle = getActualHeading();
        telemetry.addLine("Gyro turn " + direction2string(direction) + " for " + angle + " degrees");
        telemetry.addData("Start heading: ", "%.1f", startAngle);

        if(!(direction == Direction.GYRO_LEFT || direction == Direction.GYRO_RIGHT))
            throw new IllegalArgumentException("Direction should be either GYRO_LEFT or GYRO_RIGHT");

        enableGyro();
        setDirection(direction);

        double currentTurnAngle;
        setPower4WDrive(MAX_SPEED);

        do {
            currentTurnAngle = calcTurnAngleD(startAngle, getActualHeading());
        }
        while (currentTurnAngle < (angle - TURN_ERROR));
        setPower4WDrive(0.0);

        telemetry.addData("Uncorrected heading, currentTurnAngle: ", "%.1f %.1f", getAbsoluteHeading(), currentTurnAngle);

        //wait for the robot to stop spinning after power is cut off
        double spinAngle;
        do {
            spinAngle = (int) getActualHeading();
            try {
                sleep(OVERRAN_SLEEP_TIME);
            }
            catch (Exception e) {
                Log.i("IMU", "Rotation spin stabilization failure");
            }
        }
        while (spinAngle != (int) getActualHeading());

        currentTurnAngle = calcTurnAngleD(startAngle, getActualHeading());
        setPower4WDrive(NO_ERROR_SPEED * Math.signum(angle - currentTurnAngle));
        double correctionAngle = Math.abs(angle - currentTurnAngle);
        double correctionStart = getActualHeading();

        telemetry.addData("Heading stabilized, correction angle: ", "%.1f %.1f", getAbsoluteHeading(), correctionAngle);

        do {
            currentTurnAngle = calcTurnAngleD(correctionStart, getActualHeading());
        }
        while (currentTurnAngle < (correctionAngle - STOP_GAP));
        setPower4WDrive(0.0);

        telemetry.addData("Corrected heading, completed turn: ", "%.1f %.1f", getAbsoluteHeading(), currentTurnAngle);
    }

    // DO NOT USE TO ROTATE THE PLATFORM!!!
    public void gyroMove2(Direction direction, double angle, Telemetry telemetry) {
        gyroMove2(direction, angle, MAX_SPEED, telemetry);
    }

    /**
     * Calculate angle values between two directions in degrees
     * @param startAngle the initial heading
     * @param currentAngle current heading
     * @return different between initial and current heading
     */
    private double calcTurnAngleD(double startAngle, double currentAngle) {
        double turnAngle = Math.abs(currentAngle - startAngle);
        turnAngle = (turnAngle > 360) ? turnAngle - 360 : turnAngle;
        turnAngle = (turnAngle > 180) ? 360 - turnAngle : turnAngle;

        return turnAngle;
    }

    public int diagonalMove (Direction direction,double speed, double distance) {
        int wheel1Target;
        int wheel2Target;
        DcMotor motor1 = null;
        DcMotor motor2 = null;

        switch (direction){
            case DIAGONAL_RIGHT_UP:
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                motor1 = frontLeft;
                motor2 = backRight;
                break;
            case DIAGONAL_RIGHT_DOWN:
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1 = frontLeft;
                motor2 = backRight;
                break;
            case DIAGONAL_LEFT_DOWN:
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                motor1 = backLeft;
                motor2 = frontRight;
                break;
            case DIAGONAL_LEFT_UP:
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1 = backLeft;
                motor2 = frontRight;
                break;

            default:
                break;
        }

        // Determine new target position, and pass to motor controller
        wheel1Target = motor1.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        wheel2Target = motor2.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        motor1.setTargetPosition(wheel1Target);
        motor2.setTargetPosition(wheel2Target);

        // Turn On RUN_TO_POSITION
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Setting the power
        motor1.setPower(speed);
        motor2.setPower(speed);

        while (motor1.isBusy() && motor2.isBusy()) {
            // just waiting when motors are busy
        }

        // Stop all motion
        motor1.setPower(0);
        motor2.setPower(0);

        setEncoder(true);

        return wheel1Target;
    }

    //GYRO TURNING
    //this will always give postitive angle
    public double getAbsoluteHeading() {
        return Math.abs(getActualHeading());
    }

    //GYRO TURNING
    //this will give the actual angle(+/-)
    public double getActualHeading() {
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public boolean isItGold() {
        // enabling colorSensor
        //enableColorSensor();

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (colorsense.red() * SCALE_FACTOR), (int) (colorsense.green() * SCALE_FACTOR), (int) (colorsense.blue() * SCALE_FACTOR), hsvValues);
        if (hsvValues[0] < 120 && hsvValues[0] > 20) {
            // Detected a non-white color. (Probably yellow lol)
            return true;
        }
        else {
            // Detected an unsupported color. (Probably white lol)
            return false;
        }

    }

    public int linearMoveOne (DcMotor testMotor, int direction,double speed, double distance) {

        int testTarget;
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Determine new target position, and pass to motor controller
        testTarget = testMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        testMotor.setTargetPosition(testTarget);

        // Turn On RUN_TO_POSITION
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!activateSpeedProfile) {

            //Setting the power
            testMotor.setPower(Math.abs(speed));

            while (testMotor.isBusy() ) {
                // just waiting when motors are busy
            }
        }

        // Stop all motion
        testMotor.setPower(0.0);
        //setEncoder(true);

        return testTarget;
    }

    public void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
        frontLeft.setZeroPowerBehavior(behavior);
        frontLeft.setZeroPowerBehavior(behavior);
    }

    private String direction2string(Direction direction) {
        String result = "UNKNOWN";

        switch (direction)
        {
            case LEFT:
                result = "LEFT";
                break;
            case RIGHT:
                result = "RIGHT";
                break;
            case REVERSE:
                result = "REVERSE";
                break;
            case FORWARD:
                result = "FORWAR";
                break;
            case GYRO_LEFT:
                result = "GYRO-LEFT";
                break;
            case GYRO_RIGHT:
                result = "GYRO-RIGHT";
                break;

                default:
                    break;
        }

        return result;
    }

}

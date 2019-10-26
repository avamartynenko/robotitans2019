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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


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
    public IntakeMech intakeMech = null;
    public Arm frontArm = null;
    public Arm backArm = null;



    public static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.66 ;     // This is < 1.0 if geared UP
    public final double     COUNTS_PER_MOTOR_REV_WITH_GEAR    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    public final double     WHEEL_CIRCUMFRENCE      = WHEEL_DIAMETER_INCHES * 3.1415;

    public final double  MECHANUM_CPI_RATIO  = 0.5; //since it is a 4 wheel drive, you have to half the CPI

    public  final double   COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV_WITH_GEAR/ WHEEL_CIRCUMFRENCE) * MECHANUM_CPI_RATIO;



    public final int FORWARD = 1;
    public final int REVERSE = 2;
    public final int RIGHT = 3 ;
    public final int LEFT = 4;
    public final int GYRO_RIGHT = 5;
    public final int GYRO_LEFT = 6;
    public final int DIAGONAL_RIGHT_UP = 7;
    public final int DIAGONAL_RIGHT_DOWN = 8;
    public final int DIAGONAL_LEFT_UP = 9;
    public final int DIAGONAL_LEFT_DOWN = 10;





    // variables for using gryo
    Orientation angles ;
    BNO055IMU imu;




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

        if(needColorSensor) enableColorSensor();
        if(needGyro) enableGyro();

    }

    public void initTeleopModules(){

        intakeMech = new IntakeMech(hwMap);




    }

    public void setEncoder(boolean needEncoder){


        if (!needEncoder) {
            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else {

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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

    public int linearMove (int direction,double speed, double distance) {

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

        } else{
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

    public void setDirection(int direction){

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



    public void gyroMove (int direction,double speed, double angle) {

        enableGyro();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        setDirection(direction);

        //Setting the power
        while(getAbsoluteHeading() < angle) {
            setPower4WDrive(Math.abs(speed));
        }
        setPower4WDrive(0.0);

        imu.stopAccelerationIntegration();


        // Stop all motion
        // setPower4WDrive(0);

    }


    public int diagonalMove (int direction,double speed, double distance) {

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

        while (motor1.isBusy()&& motor2.isBusy()) {
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
    public double getAbsoluteHeading(){
        return Math.abs(getActualHeading());
    }

    //GYRO TURNING
    //this will give the actual angle(+/-)
    public double getActualHeading(){
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



}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmController {

    LinearOpMode linOpMode;

    public ArmController(LinearOpMode rbOp) {
        linOpMode = rbOp;
    }

    public void testArm(Arm armToTest){

        linOpMode.telemetry.addData("Status", "Start arm motions...");
        linOpMode.telemetry.update();

        armToTest.goDown(0.5);
        linOpMode.sleep(1500);

        armToTest.stop();
        linOpMode.sleep(500);


        armToTest.latchStone(0.5);
        linOpMode.sleep(1500);

        armToTest.liftUp(0.5);
        linOpMode.sleep(1500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        armToTest.dropServo.setPower(0);
        //sleep(1000);

        linOpMode.telemetry.addData("Status", "Arm motions complete");
        linOpMode.telemetry.update();
    }

    public void dropCube(Arm armToTest){

        linOpMode.telemetry.addData("Status", "Start arm motions...");
        linOpMode.telemetry.update();

        armToTest.goDown(0.5);
        linOpMode.sleep(2500);

        armToTest.stop();
        linOpMode.sleep(1000);


        armToTest.releaseStone(0.5);
        linOpMode.sleep(2500);

        armToTest.liftUp(0.5);
        linOpMode.sleep(2500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        armToTest.dropServo.setPower(0);
        linOpMode.sleep(1000);

        linOpMode.telemetry.addData("Status", "Arm motions complete");
        linOpMode.telemetry.update();
    }
}

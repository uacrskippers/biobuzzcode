package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp
public class java extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private DcMotor intake;
    private CRServo index;
    private DcMotor shooter;

    private double shooterPower = 0.5;

    private static double pprShooter = 537.7;
    private double lastTime = 0;
    private double time;
    private double ticksShooter;
    private double lastTicksShooter = 0;
    private double rpmLastShooter = 0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        left=hardwareMap.get(DcMotor.class, "leftmotor");
        right=hardwareMap.get(DcMotor.class, "rightmotor");
        intake=hardwareMap.get(DcMotor.class, "intake");
        index=hardwareMap.get(CRServo.class, "indexer");
        shooter=hardwareMap.get(DcMotor.class, "launcher");

        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            moverobot();
            runintake();
            runindexer();
            runshooter();

            if(this.gamepad1.cross) {
                positionForShooting();
            }


            telemetry.addData("Shooter Power", "%.2f %%", shooterPower * 100);

            time = System.currentTimeMillis();
            ticksShooter = shooter.getCurrentPosition();
            double dTime = time - lastTime;
            double dTicks = ticksShooter - lastTicksShooter;
            double ppsShooter = dTime / dTicks;
            double rpsShooter = (ppsShooter * 1000) / pprShooter;

            double rpsAvgShooter = (rpsShooter + rpmLastShooter) / 2;

            rpmLastShooter = rpsShooter;

            lastTime = time;
            lastTicksShooter = ticksShooter;

            telemetry.addData("Shooter RPS", "%.2f", rpsAvgShooter);
            telemetry.addData("Target RPS", "%.2f", 3.12 * shooterPower);
            telemetry.addData("Samples Per Second", "%.2f", dTime);

            telemetry.update();
        }
    }

    private void moverobot() {
        if(-this.gamepad1.left_stick_y < -0.75) {
            left.setPower(-1.0);
        }
        else if(Math.abs(this.gamepad1.left_stick_y) < 0.1) {
            left.setPower(0.0);
        }
        else if(-this.gamepad1.left_stick_y > 0.75) {
            left.setPower(1.0);
        }
        else {
            left.setPower(-this.gamepad1.left_stick_y);
        }
        if(this.gamepad1.right_stick_y < -0.75) {
            right.setPower(-1.0);
        }
        else if(Math.abs(this.gamepad1.right_stick_y) < 0.1) {
            right.setPower(0.0);
        }
        else if(this.gamepad1.right_stick_y > 0.75) {
            right.setPower(1.0);
        }
        else {
            right.setPower(this.gamepad1.right_stick_y);
        }
    }

    private void runintake() {
        if(this.gamepad1.right_trigger > 0.75) {
            intake.setPower(1.0);
        }
        else if(this.gamepad1.right_trigger > 0.1) {
            intake.setPower(this.gamepad1.right_trigger);
        }
        else if(this.gamepad1.left_trigger > 0.1) {
            intake.setPower(-1.0);
        }
        else {
            intake.setPower(0.0);
        }
    }

    private void runindexer() {
        if(this.gamepad1.right_bumper) {
            index.setPower(-1.0);
        }
        else if(this.gamepad1.left_bumper) {
            index.setPower(1.0);
        }
        else {
            index.setPower(0.0);
        }
    }

    private void runshooter() {
        if(this.gamepad1.triangleWasReleased()) {
            shooterPower+=0.05;
            if(shooterPower>1) {
                shooterPower=1;
            }
        }
        if(this.gamepad1.squareWasReleased()) {
            shooterPower-=0.01;
            if(shooterPower<0) {
                shooterPower=0;
            }
        }
        shooter.setPower(shooterPower);
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void positionForShooting() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                telemetry.addData("Detected: ", singleDet.id);
                telemetry.addData("Distance: ", singleDet.ftcPose.range);
                telemetry.addData("Bearing: ", singleDet.ftcPose.bearing);

                if (singleDet.ftcPose.bearing > 0.1) {
                    left.setPower(.5);
                    right.setPower(-.5);
                } else if (singleDet.ftcPose.bearing < -0.1) {
                    left.setPower(-.5);
                    right.setPower(.5);
                }
                if (singleDet.ftcPose.range > 0.1) {
                    left.setPower(.5);
                    right.setPower(.5);
                } else if (singleDet.ftcPose.range < 0.1) {
                    left.setPower(-.5);
                    right.setPower(-.5);
                }
            }
        }
    }
}
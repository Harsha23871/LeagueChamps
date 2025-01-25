package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="4 Sample Markers")
public class Four_Sample_Markers extends LinearOpMode {
    public DcMotor elevator, armMotor = null;
    public Servo claw, wrist, bucket , intakeClaw, intake_extension= null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        intake_extension = hardwareMap.get(Servo.class, "intake_extension");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);



//        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        bucket.setPosition(0.2);


        waitForStart();

//        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -64.00, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    intake_extension.setPosition(0);
                    wrist.setPosition(0);
                    armMotor.setTargetPosition(1100);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.7);
                    intakeClaw.setPosition(0.5);

                    elevator.setTargetPosition(3400);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.7);
                })
                .lineToLinearHeading(new Pose2d(-59, -61, Math.toRadians(45.00)))
                .build();                         //-57   //-59




        TrajectorySequence FirstGrab = drive.trajectorySequenceBuilder(trajectory0.end())
                .lineToLinearHeading(new Pose2d(-52, -45, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(1300);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    intakeClaw.setPosition(1);
                    wrist.setPosition(0.5);//releasing position!!!!
                    armMotor.setTargetPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeClaw.setPosition(0.5);
                    wrist.setPosition(0);
                    armMotor.setTargetPosition(1100);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    elevator.setTargetPosition(3400);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(0.7);
                })

                .build();

        TrajectorySequence FirstScore = drive.trajectorySequenceBuilder(FirstGrab.end())
                .lineToLinearHeading(new Pose2d(-59, -61, Math.toRadians(45)))
                .build();                         //-57   //-59

        TrajectorySequence SecondGrab = drive.trajectorySequenceBuilder(FirstScore.end())
                .lineToLinearHeading(new Pose2d(-62, -44.5, Math.toRadians(90.00)))
                .build();

        TrajectorySequence SecondScore = drive.trajectorySequenceBuilder(SecondGrab.end())
                .lineToLinearHeading(new Pose2d(-59, -61, Math.toRadians(45)))
                .build();                         //-57   //-59

        TrajectorySequence ThirdGrab = drive.trajectorySequenceBuilder(SecondScore.end())
                .lineToLinearHeading(new Pose2d(-62, -44, Math.toRadians(120)))
                .build();

        TrajectorySequence ThirdScore = drive.trajectorySequenceBuilder(ThirdGrab.end())
                .lineToLinearHeading(new Pose2d(-59, -61, Math.toRadians(45)))
                .build();                         //-57   //-59
//
//


// add function similar to a trajectory but for arm and wrist motors like function close()







//        intake_extension.setPosition(0);
//        wrist.setPosition(0);
//        armMotor.setTargetPosition(1100);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.7);
//        intakeClaw.setPosition(0.5);
//        //  sleep(1000);
//
//        elevator.setTargetPosition(3400);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.7);
        drive.followTrajectorySequence(trajectory0);
        ///////////////
        sleep(1000); // 1000
        ////////////
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(1);

        //////////////////////
        sleep(1000); //1000
        /////////////////////

        bucket.setPosition(0.2);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
        // 1 is close 0.7 is open


        drive.followTrajectorySequence(FirstGrab);

        //////////////////////////////
        sleep(1000); // 1000
        /////////////////////////////


        drive.followTrajectorySequence(FirstScore);

        ////////////////////////////////
        sleep(1000); // 1000
        ///////////////////////////////////

        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(1);
        sleep(1000);
        bucket.setPosition(0.2);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);

        drive.followTrajectorySequence(SecondGrab);
        sleep(1000);
        armMotor.setTargetPosition(1300);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(500); //1000
        intakeClaw.setPosition(1);
        sleep(500);
        wrist.setPosition(0.5);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        intakeClaw.setPosition(0.5);
        sleep(500);
        wrist.setPosition(0);

        armMotor.setTargetPosition(1100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        elevator.setTargetPosition(3400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
        drive.followTrajectorySequence(SecondScore);
        sleep(1000);
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(1);
        sleep(1000);
        bucket.setPosition(0.2);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);

        drive.followTrajectorySequence(ThirdGrab);
        sleep(1000);//1000
        armMotor.setTargetPosition(1300);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        sleep(1000);
        intakeClaw.setPosition(1);
        sleep(500);
        wrist.setPosition(0.5);//releasing position!!!!
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        intakeClaw.setPosition(0.5);
        sleep(500);
        wrist.setPosition(0);
        armMotor.setTargetPosition(1000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);


        elevator.setTargetPosition(3400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
        drive.followTrajectorySequence(ThirdScore);
        sleep(1000);

        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(1);
        sleep(1000);
        bucket.setPosition(0.2);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.8);






















        if (isStopRequested()) return;
    }}
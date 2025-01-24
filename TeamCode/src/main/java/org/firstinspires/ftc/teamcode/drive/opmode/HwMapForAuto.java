//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class HwMapForAuto {
//
//    private int elevator_resting_pos = 0;
//    private int elevator_scoring_pos = 1800;
//
//    private HardwareMap hw = null;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    public DcMotor elevator = null;
//
//    // Constructor for the HwMapForAuto class
//    public HwMapForAuto(HardwareMap hardwareMap) {
//        this.hw = hardwareMap;
//
//        // Initialize the elevator motor
//        elevator = hw.get(DcMotor.class, "elevator_motor");
//        elevator.setDirection(DcMotor.Direction.REVERSE);
//        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    // Moves the elevator motor to the scoring position
//    public void elevator_Scoring_Pos() {
//        elevator.setTargetPosition(elevator_scoring_pos);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.5);
//    }
//
//    // Moves the elevator motor to the resting position
//    public void elevator_Resting_Pos() {
//        elevator.setTargetPosition(elevator_resting_pos);
//        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevator.setPower(0.5);
//    }
//}

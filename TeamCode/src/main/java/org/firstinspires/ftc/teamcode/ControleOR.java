package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp
public class ControleOR extends LinearOpMode {
    private Limelight3A limelight; // Variável da câmera
    private LLResult result;
    private IMU imu;
    private DcMotor dd, de, td, te;// Variáveis dos motores
    private DcMotor la, lk; // Variáveis do shooter
    private Boolean lado = null; // null = não lido // true = azul // false = vermelho

    @Override
    public void runOpMode() {
        // Mapear os motores
        dd = hardwareMap.get(DcMotor.class, "dd"); // 0
        de = hardwareMap.get(DcMotor.class, "de"); // 1
        td = hardwareMap.get(DcMotor.class, "td"); // 2
        te = hardwareMap.get(DcMotor.class, "te"); // 3

        la = hardwareMap.get(DcMotor.class, "la"); //
        lk = hardwareMap.get(DcMotor.class, "lk"); //

        imu = hardwareMap.get(IMU.class, "imu"); //

        //Corrige a direção dos motores
        de.setDirection(DcMotor.Direction.REVERSE);
        te.setDirection(DcMotor.Direction.REVERSE);
        lk.setDirection(DcMotor.Direction.REVERSE);

        //Orientação do Robô
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revOrientation));

        // Inicia e configura a câmera
        initCamera();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            fullTelemetry();
            //Chassis
            drive(-gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);


            //Lançador

        }
    }
    public void drive (double y, double x, double turn){
        double teta = Math.atan2(y,x);
        double power = Math.hypot(x,y);
        double sin = Math.sin(teta - Math.PI/4);
        double cos = Math.cos(teta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        double dep = power * cos/max + turn;
        double ddp = power * sin/max - turn;
        double tep = power * sin/max + turn;
        double tdp = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1){
            dep /= power + turn;
            ddp /= power + turn;
            tep /= power + turn;
            tdp /= power + turn;
        }

        de.setPower(dep);
        dd.setPower(ddp);
        te.setPower(tep);
        td.setPower(tdp);
    }
    public void fullTelemetry(){
        telemetry.addData("Status", "Running");

        if (lado == null){
            telemetry.addLine("Aliança atual: UNKNOWN");
        }else if(lado){
            telemetry.addLine("Aliança atual: AZUL");
        }else{
            telemetry.addLine("Aliança atual: VERMELHO");
        }

        telemtryCamera();
        telemetry.addData("Heading",getHeading());

        telemetry.update();
    }
    public Double[] leAngulos (){
        Double x = null;
        Double y = null;
        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int tag;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tag = fr.getFiducialId();
                if((tag == 20 ^ tag == 24) && lado != null){ // ^ = xor
                    x = fr.getTargetXDegrees();
                    y = fr.getTargetYDegrees();
                }
            }
        }
        Double[] xy = new Double[2];
        xy[0] = x;
        xy[1] = y;
        return xy;
    }
    public void telemtryCamera () {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int tag;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tag = fr.getFiducialId();
                telemetry.addData("Tag Detectada", true);
                telemetry.addData("AprilTag ID", tag);

                telemetry.addData("X (°)", "%.2f", fr.getTargetXDegrees());
                telemetry.addData("Y (°)", "%.2f", fr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Tag Detectada", false);
            telemetry.addLine("Nenhuma AprilTag visível.");
        }
    }
    public void leLado () {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int tag;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tag = fr.getFiducialId();
                if (tag == 20 && lado == null){
                    lado = true;
                }else if (tag == 24 && lado == null){
                    lado = false;
                }
            }
        }
    }
    public void initCamera(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
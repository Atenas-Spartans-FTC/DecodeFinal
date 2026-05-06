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
public class ControleOA extends LinearOpMode {
    private Limelight3A limelight; // Variável da câmera
    private LLResult result;
    private IMU imu;
    private DcMotor dd, de, td, te;// Variáveis dos motores
    private DcMotor l1, l2;
    private Boolean lado = null; // null = não lido // true = azul // false = vermelho
    double sin, cos, teta, power, max, dep, ddp, tep, tdp;

    @Override
    public void runOpMode() {
        // Mapear os motores
        dd = hardwareMap.get(DcMotor.class, "dd"); // 0
        de = hardwareMap.get(DcMotor.class, "de"); // 1
        td = hardwareMap.get(DcMotor.class, "td"); // 2
        te = hardwareMap.get(DcMotor.class, "te"); // 3
        l1 = hardwareMap.get(DcMotor.class, "l1"); // exp 1
        l2 = hardwareMap.get(DcMotor.class, "l2"); // exp 2

        imu = hardwareMap.get(IMU.class, "imu"); //

        //Corrige a direção dos motores
        de.setDirection(DcMotor.Direction.REVERSE);
        te.setDirection(DcMotor.Direction.REVERSE);

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
            drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, getHeadingR());

            if(gamepad1.a){
                l1.setPower(1);
                l2.setPower(1);
            }else if (gamepad1.b){
                l1.setPower(-1);
                l2.setPower(-1);
            }else{
                l1.setPower(0);
                l2.setPower(0);
            }

            //Lançador

        }
    }
    public void drive(double y, double x, double turn, double rad) {
        // 1. Corrige o joystick usando o ângulo do robô (field-centric)
        double cosA = Math.cos(rad);
        double sinA = Math.sin(rad);

        double xRot = x * cosA - y * sinA;
        double yRot = x * sinA + y * cosA;

        // 2. Converte o vetor de movimento em ângulo e intensidade
        double teta = Math.atan2(yRot, xRot);
        double power = Math.hypot(xRot, yRot);

        // 3. Ajuste de 45° para rodas mecanum
        double sin = Math.sin(teta - Math.PI / 4);
        double cos = Math.cos(teta - Math.PI / 4);

        // 4. Normalização
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // 5. Cálculo da potência de cada motor
        double dep = power * cos / max + turn; // dianteiro esquerdo
        double ddp = power * sin / max - turn; // dianteiro direito
        double tep = power * sin / max + turn; // traseiro esquerdo
        double tdp = power * cos / max - turn; // traseiro direito

        // 6. Garante que nenhuma potência passe de 1
        if ((power + Math.abs(turn)) > 1) {
            dep /= power + Math.abs(turn);
            ddp /= power + Math.abs(turn);
            tep /= power + Math.abs(turn);
            tdp /= power + Math.abs(turn);
        }

        // 7. Envia potência para os motores
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
        telemetry.addData("Heading", getHeadingD());

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
    public double getHeadingD(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public double getHeadingR(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
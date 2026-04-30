package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutonomoTG extends LinearOpMode {
    private DcMotor dd,de,td,te;
    GoBildaPinpointDriver pinpointer;

    private final double kp = 0.000077;//verificar no robo
    private final double ki = 0.0001;//verificar no robo
    private final double kd = 0.0000003;

    // PID de heading (normalmente mais forte)
    private final double kpH = 0.01;
    private final double kiH = 0.0;
    private final double kdH = 0.0005;

    private final double ilimit = 500;

    double errorsumX = 0, errorsumY = 0, errorsumH = 0;
    double lastErrorX = 0, lastErrorY = 0, lastErrorH = 0;

    boolean chegou;
    final double positionTolerance = 50;     // encoder ticks (ajuste no robô)
    final double headingTolerance = Math.toRadians(2); // 2 graus
    final double powerTolerance = 0.05;       // potência mínima

    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        dd = hardwareMap.get(DcMotor.class, "dd");
        de = hardwareMap.get(DcMotor.class, "de");
        td = hardwareMap.get(DcMotor.class, "td");
        te = hardwareMap.get(DcMotor.class, "te");

        pinpointer = hardwareMap.get(GoBildaPinpointDriver.class, "pn");

        de.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpointer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpointer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );// x,y

        pinpointer.resetPosAndIMU();

        pinpointer.update();
        final double lastX = pinpointer.getEncoderX();
        final double lastY = pinpointer.getEncoderY();

        errorsumX = 0;
        lastErrorX = 0;
        errorsumY = 0;
        lastErrorY = 0;
        errorsumH = 0;
        lastErrorH = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            pinpointer.update();

            double x = pinpointer.getEncoderX() - lastX;
            double y = pinpointer.getEncoderY() - lastY;
            double heading = pinpointer.getHeading(AngleUnit.RADIANS);

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", heading);

            chegou = goToPID(
                    10000,
                    10000,
                    0,
                    x,
                    y,
                    heading
            );

            telemetry.update();
        }
    }
    public boolean goToPID(double setpointX, double setpointY, double setpointHeading, double positionX, double positionY, double heading) {
        double dt = time.seconds();
        time.reset();

        // ---------- ERROS ----------
        double errorX = setpointX - positionX;
        double errorY = setpointY - positionY;

        double errorH = setpointHeading - heading;
        errorH = Math.atan2(Math.sin(errorH), Math.cos(errorH));

        // ---------- INTEGRAIS ----------
        if (Math.abs(errorX) < ilimit) errorsumX += errorX * dt;
        if (Math.abs(errorY) < ilimit) errorsumY += errorY * dt;
        if (Math.abs(errorH) < ilimit) errorsumH += errorH * dt;

        // ---------- DERIVADAS ----------
        double dX = (errorX - lastErrorX) / dt;
        double dY = (errorY - lastErrorY) / dt;
        double dH = (errorH - lastErrorH) / dt;

        lastErrorX = errorX;
        lastErrorY = errorY;
        lastErrorH = errorH;

        // ---------- PID ----------
        double xPower = kp * errorX + ki * errorsumX + kd * dX;
        double yPower = kp * errorY + ki * errorsumY + kd * dY;
        double turnPower = kpH * errorH + kiH * errorsumH + kdH * dH;

        // ---------- MECANUM ----------
        drive(yPower, xPower, turnPower);

        // ---------- CONDIÇÃO DE CHEGADA ----------
        boolean positionOk =
                Math.abs(errorX) < positionTolerance &&
                        Math.abs(errorY) < positionTolerance;

        boolean headingOk =
                Math.abs(errorH) < headingTolerance;

        boolean powerOk =
                Math.abs(xPower) < powerTolerance &&
                        Math.abs(yPower) < powerTolerance &&
                        Math.abs(turnPower) < powerTolerance;

        boolean arrived = positionOk && headingOk && powerOk;

        // ---------- TELEMETRIA ----------
        telemetry.addData("Erro X", errorX);
        telemetry.addData("Erro Y", errorY);
        telemetry.addData("Erro Heading", errorH);
        telemetry.addData("Soma Erro X",errorsumX);
        telemetry.addData("Soma Erro Y",errorsumY);
        telemetry.addData("Soma Erro Heading",errorsumH);
        telemetry.addData("Derivada X",errorsumX);
        telemetry.addData("Derivada Y",errorsumY);
        telemetry.addData("Derivada Heading",errorsumH);
        telemetry.addData("Power X", xPower);
        telemetry.addData("Power Y", yPower);
        telemetry.addData("Power Turn", turnPower);
        telemetry.addData("Chegou", arrived);

        return arrived;
    }
    public void drive(double y, double x, double turn) {

        double teta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(teta - Math.PI / 4);
        double cos = Math.cos(teta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double dep = power * cos / max + turn;
        double ddp = power * sin / max - turn;
        double tep = power * sin / max + turn;
        double tdp = power * cos / max - turn;

        double scale = Math.max(1, power + Math.abs(turn));

        de.setPower(dep / scale);
        dd.setPower(ddp / scale);
        te.setPower(tep / scale);
        td.setPower(tdp / scale);
    }
}

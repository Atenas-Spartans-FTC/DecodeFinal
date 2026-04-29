package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.sql.Timestamp;
import java.util.Timer;

@Autonomous
public class AutonomoTG extends LinearOpMode {
    private DcMotor dd,de,td,te;
    GoBildaPinpointDriver pinpointer;
    private final double kp = 0.000077;//verificar no robo
    private final double ki = 0.0001;//verificar no robo
    private final double kd = 0.0000003;
    double setpoint = 0, errorsum = 0, ilimit = 500, lasterror = 0;
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

        errorsum = 0;
        lasterror = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            pinpointer.update();

            double x = pinpointer.getEncoderX() - lastX;
            double y = pinpointer.getEncoderY() - lastY;
            double z = pinpointer.getHeading(AngleUnit.DEGREES);

            setpoint = 10000;

            double error = setpoint - y;
            if (Math.abs(error) < ilimit ) {
                errorsum += error * time.seconds();
            }
            double errorrate = (error - lasterror) / time.seconds();

            time.reset();
            lasterror = error;

            double m = 640;
            if (error < 0 && errorsum > 0){
                errorsum = -m;
            }else if (error > 0 && errorsum < 0){
                errorsum = m;
            }

            double speed = 0;
            if(error != 0){
                speed = kp * error + ki * errorsum + kd * errorrate;
            }

            dd.setPower(speed);
            de.setPower(speed);
            td.setPower(speed);
            te.setPower(speed);

            telemetry.addData("Odometria Y",y);
            telemetry.addData("Odometria X",x);
            telemetry.addData("Odometria Z",z);
            telemetry.addData("Error", error);
            telemetry.addData("Erro Sum", errorsum);
            telemetry.addData("Erro Rate", errorrate);
            telemetry.addData("Poder", speed);
            telemetry.update();
        }
    }
}

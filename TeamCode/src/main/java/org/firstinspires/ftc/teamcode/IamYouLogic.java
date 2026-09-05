package org.firstinspires.ftc.teamcode;

public class IamYouLogic {

    // ---- Tunable constants ----
    public static final double JOYSTICK_DEADZONE    = 0.05;
    public static final double SLOW_MODE_DRIVE_MULT = 0.35;
    public static final double SLOW_MODE_TWIST_MULT = 0.50;

    private static final double HEADING_NORTH =   0.0;
    private static final double HEADING_EAST  = -90.0;
    private static final double HEADING_SOUTH = 180.0;
    private static final double HEADING_WEST  =  90.0;

    public static double driverOffsetRadians(String pos) {
        switch (pos) {
            case "EAST":  return Math.toRadians(-90.0);
            case "NORTH": return Math.toRadians(180.0);
            case "WEST":  return Math.toRadians(90.0);
            case "SOUTH":
            default:      return 0.0;
        }
    }

    public static class DriveParameters {
        public double leftStickY;
        public double leftStickX;
        public double rightStickX;
        public double leftTrigger;
        public double rawHeading;
        public String initCardinal;
        public String driverPosition;

        public DriveParameters(double leftStickY, double leftStickX, double rightStickX,
                               double leftTrigger, double rawHeading, String initCardinal,
                               String driverPosition) {
            this.leftStickY = leftStickY;
            this.leftStickX = leftStickX;
            this.rightStickX = rightStickX;
            this.leftTrigger = leftTrigger;
            this.rawHeading = rawHeading;
            this.initCardinal = initCardinal;
            this.driverPosition = driverPosition;
        }
    }

    public static class DriveResult {
        public ChassisMotors powers;
        public double correctedHeadingRadians;
        public double fieldDrive;
        public double fieldStrafe;
        public double drive;
        public double strafe;
        public double twist;

        public DriveResult(ChassisMotors powers, double correctedHeadingRadians,
                           double fieldDrive, double fieldStrafe,
                           double drive, double strafe, double twist) {
            this.powers = powers;
            this.correctedHeadingRadians = correctedHeadingRadians;
            this.fieldDrive = fieldDrive;
            this.fieldStrafe = fieldStrafe;
            this.drive = drive;
            this.strafe = strafe;
            this.twist = twist;
        }
    }

    public DriveResult calculate(DriveParameters params) {
        // ================= HEADING READ =================
        double rawHeading = params.rawHeading;
        String initCardinal = params.initCardinal;

        double correctedHeading;
        if (initCardinal.equals("NORTH")) {
            correctedHeading = Math.toRadians(rawHeading - HEADING_NORTH);
        } else if (initCardinal.equals("EAST")) {
            correctedHeading = Math.toRadians(rawHeading - HEADING_EAST);
        } else if (initCardinal.equals("SOUTH")) {
            correctedHeading = Math.toRadians(rawHeading - HEADING_SOUTH);
        } else {
            // WEST
            correctedHeading = Math.toRadians(rawHeading - HEADING_WEST);
        }

        // ================= GAMEPAD READ =================
        double rawDrive  = -params.leftStickY;
        double rawStrafe =  params.leftStickX;
        double twist     = applyDeadzone(-params.rightStickX, JOYSTICK_DEADZONE);

        double[] trans = applyDeadzoneVector(rawDrive, rawStrafe, JOYSTICK_DEADZONE);
        double drive   = trans[0];
        double strafe  = trans[1];

        // ================= DRIVER POSITION ROTATION =================
        double doff  = driverOffsetRadians(params.driverPosition);
        double cosD  = Math.cos(doff);
        double sinD  = Math.sin(doff);
        double driveR  = drive * cosD - strafe * sinD;
        double strafeR = drive * sinD + strafe * cosD;
        drive  = driveR;
        strafe = strafeR;

        // ================= PER-SEGMENT INPUT CORRECTION =================
        double d, s;
        if (initCardinal.equals("NORTH")) {
            d =  drive;   s = -strafe;
        } else if (initCardinal.equals("SOUTH")) {
            d = -drive;   s =  strafe;
        } else if (initCardinal.equals("EAST")) {
            d =  strafe;  s =  drive;
        } else {
            // WEST
            d = -strafe;  s = -drive;
        }

        // ================= FIELD-CENTRIC ROTATION MATRIX =================
        double cosH = Math.cos(correctedHeading);
        double sinH = Math.sin(correctedHeading);

        double fieldDrive  =  d * cosH + s * sinH;
        double fieldStrafe = -d * sinH + s * cosH;

        // ================= SLOW MODE =================
        double driveMult = 1.0;
        double twistMult = 1.0;
        if (params.leftTrigger > 0.1) {
            driveMult = SLOW_MODE_DRIVE_MULT;
            twistMult = SLOW_MODE_TWIST_MULT;
        }

        // ================= MECANUM KINEMATICS =================
        double denominator = Math.max(
                Math.abs(fieldDrive) + Math.abs(fieldStrafe) + Math.abs(twist), 1.0);

        double normDrive  = (fieldDrive  / denominator) * driveMult;
        double normStrafe = (fieldStrafe / denominator) * driveMult;
        double normTwist  = (twist       / denominator) * twistMult;

        double flPower = normDrive + normStrafe + normTwist;
        double blPower = normDrive - normStrafe + normTwist;
        double frPower = normDrive - normStrafe - normTwist;
        double brPower = normDrive + normStrafe - normTwist;

        return new DriveResult(
                new ChassisMotors(flPower, frPower, blPower, brPower),
                correctedHeading,
                fieldDrive,
                fieldStrafe,
                drive,
                strafe,
                twist
        );
    }

    public String snapToCardinal(double headingDeg) {
        while (headingDeg >  180.0) headingDeg -= 360.0;
        while (headingDeg <= -180.0) headingDeg += 360.0;

        if      (headingDeg >= -45.0  && headingDeg <  45.0)  return "NORTH";
        else if (headingDeg >=  45.0  && headingDeg < 135.0)  return "WEST";
        else if (headingDeg >= -135.0 && headingDeg < -45.0)  return "EAST";
        else                                                    return "SOUTH";
    }

    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0.0 : value;
    }

    private double[] applyDeadzoneVector(double x, double y, double deadzone) {
        return Math.hypot(x, y) < deadzone ? new double[]{0.0, 0.0} : new double[]{x, y};
    }
}

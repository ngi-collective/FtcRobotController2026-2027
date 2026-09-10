package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertSame;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Pins the drivetrain wiring (hardware-map names, direction, run mode) to a
 * single expected configuration. This is the one place a physical rewire or a
 * copy-pasted OpMode could silently reintroduce the FL/BL vs FR/BR direction
 * mismatch that used to exist between BasicMecanumTeleOp and iamyou.
 */
class DriveHardwareTest {

    private HardwareMap hardwareMap;
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        fl = mock(DcMotorEx.class);
        fr = mock(DcMotorEx.class);
        bl = mock(DcMotorEx.class);
        br = mock(DcMotorEx.class);

        when(hardwareMap.get(DcMotorEx.class, "FL")).thenReturn(fl);
        when(hardwareMap.get(DcMotorEx.class, "FR")).thenReturn(fr);
        when(hardwareMap.get(DcMotorEx.class, "BL")).thenReturn(bl);
        when(hardwareMap.get(DcMotorEx.class, "BR")).thenReturn(br);
    }

    @Test
    void initWithEncoders_mapsCanonicalDeviceNames() {
        DriveHardware drive = DriveHardware.initWithEncoders(hardwareMap);

        assertSame(fl, drive.fl);
        assertSame(fr, drive.fr);
        assertSame(bl, drive.bl);
        assertSame(br, drive.br);
    }

    @Test
    void initWithEncoders_setsDirectionsMatchingRobotWiring() {
        DriveHardware.initWithEncoders(hardwareMap);

        verify(fl).setDirection(DcMotorSimple.Direction.REVERSE);
        verify(bl).setDirection(DcMotorSimple.Direction.REVERSE);
        verify(fr).setDirection(DcMotorSimple.Direction.FORWARD);
        verify(br).setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Test
    void initWithEncoders_usesEncoderModeAndBrakeOnZeroPower() {
        DriveHardware.initWithEncoders(hardwareMap);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            verify(m).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verify(m).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Test
    void initWithoutEncoders_usesRunWithoutEncoderAndLeavesZeroPowerBehaviorUntouched() {
        DriveHardware.initWithoutEncoders(hardwareMap);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            verify(m).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verify(m, never()).setZeroPowerBehavior(org.mockito.ArgumentMatchers.any());
        }
    }

    @Test
    void setPowers_appliesEachWheelToItsOwnMotor() {
        DriveHardware drive = DriveHardware.initWithEncoders(hardwareMap);

        drive.setPowers(new ChassisMotors(0.1, 0.2, 0.3, 0.4));

        verify(fl).setPower(0.1);
        verify(fr).setPower(0.2);
        verify(bl).setPower(0.3);
        verify(br).setPower(0.4);
    }

    @Test
    void stop_zeroesAllFourMotors() {
        DriveHardware drive = DriveHardware.initWithEncoders(hardwareMap);

        drive.stop();

        verify(fl).setPower(0);
        verify(fr).setPower(0);
        verify(bl).setPower(0);
        verify(br).setPower(0);
    }
}

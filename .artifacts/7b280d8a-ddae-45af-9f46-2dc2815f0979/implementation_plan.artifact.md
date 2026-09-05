# Implementation Plan - Unit Tests for iamyou.java

The goal is to add unit tests for the logic in `iamyou.java` following the pattern established in `MecanumMovementTest.java`. This involves extracting the core calculation logic from the `LinearOpMode` into a testable class.

## Proposed Changes

### Logic Extraction

#### [NEW] [IamYouLogic.java](file:///Users/abhinavanand/development/src/FtcRobotController2026-2027/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/IamYouLogic.java)
Create a new class that encapsulates the drive calculations, including:
- Heading correction based on initial cardinal direction.
- Driver position offset rotation.
- Per-segment input correction.
- Field-centric rotation matrix.
- Slow mode scaling.
- Mecanum kinematics.
- Deadzone application.
- Cardinal snapping.

#### [MODIFY] [iamyou.java](file:///Users/abhinavanand/development/src/FtcRobotController2026-2027/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/iamyou.java)
Refactor `iamyou.java` to use `IamYouLogic` for its calculations, reducing duplication and making the `OpMode` cleaner.

### Testing

#### [NEW] [IamYouLogicTest.java](file:///Users/abhinavanand/development/src/FtcRobotController2026-2027/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/IamYouLogicTest.java)
Create a unit test class using JUnit 5 (as in `MecanumMovementTest.java`) to verify the logic in `IamYouLogic`. Tests will cover:
- `snapToCardinal` accuracy.
- `calculateCorrectedHeading` for all cardinal starts.
- Drive power calculations for various combinations of:
    - Robot heading.
    - Initial cardinal direction.
    - Driver position.
    - Joystick inputs.
    - Slow mode status.

## Verification Plan

### Automated Tests
- Run the new unit tests using Gradle: `./gradlew :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.IamYouLogicTest`
- Ensure existing tests still pass: `./gradlew :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.MecanumMovementTest`

### Manual Verification
- Deploy the refactored `iamyou.java` to the robot and verify that it still drives as expected in all modes.

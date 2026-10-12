# Team 25805 BioBuzz: Robot Code Guide

Welcome to the team's robot code! This page explains how the code is organized so you
can find things quickly and add to it without breaking it.

## The big idea

Our code is split into three kinds of pieces. Each piece has one job:

| Piece | Job | Where it lives |
|-------|-----|----------------|
| **Op modes** | Decide *what the robot should do* (read the gamepad, run an autonomous routine) | `teamcode/` |
| **RobotHardware** | Knows *what parts the robot has* and how they are wired and set up | `teamcode/RobotHardware.java` |
| **Mechanisms** | Know *how to use a part* (the drivetrain, the IMU, an arm...) | `teamcode/mechanisms/` |

```
 Op mode  ──asks──▶  RobotHardware  ──holds──▶  Mechanisms  ──use──▶  Motors & sensors
(the driver)       (the robot)               (drive, imu, ...)
```

**An op mode never touches a motor directly.** It only talks to `robot.drive`, `robot.imu`,
and so on. That way, if a motor gets rewired or renamed, we fix it in one place and every
op mode keeps working.

## Files

```
teamcode/
├── RobotHardware.java          ← all names, settings, and motor/sensor setup
├── RobotTeleopMecanumDrive.java ← driver-controlled op mode
└── mechanisms/
    ├── MecanumDrive.java       ← turns forward/strafe/turn into 4 wheel powers
    ├── ImuSensor.java          ← tells us which way the robot is facing
    └── PinpointOdometry.java   ← tells us where the robot is on the field (goBILDA Pinpoint)
```

## How an op mode uses the robot

```java
@TeleOp(name = "My TeleOp", group = "Robot")
public class MyTeleop extends OpMode {
    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);   // sets up everything
    }

    @Override
    public void loop() {
        robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    @Override
    public void stop() {
        robot.stop();                             // stop all motors
    }
}
```

## Common jobs

### Change a setting (name, speed, direction)
Open `RobotHardware.java`. The names and tuning values are at the top, and the motor setup
(directions, encoder modes) is in the constructor.

- **A motor name changed in the Driver Station config:** edit the `..._NAME` constants.
- **The robot is too fast for an outreach event:** lower `MAX_DRIVE_SPEED` (0.0 to 1.0).
- **A wheel spins the wrong way:** flip its `setDirection(...)` line in the constructor.
- **Robot turns the wrong way for field-relative driving:** check `HUB_LOGO_DIRECTION` and
  `HUB_USB_DIRECTION` match how the Control Hub is mounted.

### Add a new mechanism (arm, claw, intake, shooter...)
1. **Write the class** in `mechanisms/`. It takes its motors or sensors in its
   **constructor** and has methods that say what it does (`raise()`, `grab()`, ...).
   It never calls `hardwareMap` and never uses a config name. Use `MecanumDrive` as a model.
2. **Add its settings** (names, speeds, limits) to the top of `RobotHardware`.
3. **Set it up in the `RobotHardware` constructor:** get the motors from `hardwareMap`,
   configure them, and create the mechanism.
4. **Add a public field**, e.g. `public final Arm arm;`
5. **Stop it safely:** add a line to `RobotHardware.stop()` if it moves.
6. **Use it in an op mode:** `robot.arm.raise();`

### Add a new op mode
Copy `RobotTeleopMecanumDrive.java`, rename it, and change the `@TeleOp(name = ...)` text.
Remove the `@Disabled` line so it shows up on the Driver Station.

## Rules of thumb

- **One job per class.** If a class is doing two things, split it.
- **Names say what things do.** `driveFieldRelative` beats `dfr`.
- **No magic numbers.** Put a number with a meaning (a speed, a position) in a named constant
  in `RobotHardware`, not buried in the middle of a method.
- **Comment the *why*,** not just the *what*.
- **Test on the robot in small steps.** Change one thing, try it, then move on.

---

# FTC SDK Notes

*The rest of this file is the standard information that comes with the FTC SDK.*

## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. The process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""
# RoboPeda Rebuilt Robot – Calibration, Tune & Measure Checklist

Use this when the robot is done and before competition. Do **measure** and **tune** in this order where dependencies exist.

---

## 1. Hardware / Measure (physical)

| Item | What to do | Where it goes |
|------|------------|----------------|
| **Spindexer** | Measure one full rotation in **encoder ticks**. Compute `TICKS_PER_SLOT = ticks_per_rev / 3`. Find encoder position where one slot is under HuskyLens → set `ENCODER_ZERO_OFFSET_TICKS` so slot 0 aligns. | [Spindexer.java](src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/Spindexer.java) |
| **Turret** | Measure one full rotation in **encoder ticks**. Compute `TURRET_TICKS_PER_DEGREE = ticks_per_rev / 360`. Measure mechanical limits → set `TURRET_MIN_DEG`, `TURRET_MAX_DEG`. With robot facing goal, set turret to 0° and note offset → `TURRET_ZERO_OFFSET_DEG`. | [TurretedShooter.java](src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/TurretedShooter.java) |
| **Turret AprilTag scan** | With robot in **auto start pose**, rotate turret until the AprilTag motif is in view. Record that angle → `TURRET_ANGLE_APRILTAG_SCAN_DEG`. | [TurretedShooter.java](src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/TurretedShooter.java) |
| **Pinpoint pods** | Measure pod offsets from robot center (forward pod Y, strafe pod X) in **inches**. Confirm forward encoder increases when robot goes forward, strafe when robot goes left; reverse in config if not. | [Constants.java](src/main/java/org/firstinspires/ftc/teamcode/config/pedro/Constants.java), [PinpointLocalizerSubsystem.java](src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/PinpointLocalizerSubsystem.java) |
| **Goal position** | Define field origin (e.g. center, or your auto start). Measure goal center in that frame (inches). Set **Red** `GOAL_X_IN`, `GOAL_Y_IN` and **Blue** (mirrored). | Red/Blue TeleOp OpModes |
| **Goal height** | Measure height of **goal center** above the **shooter launch point** (inches) → `GOAL_HEIGHT_ABOVE_LAUNCH_IN`. | Red/Blue TeleOp OpModes |

---

## 2. Vision

| Item | What to do | Where |
|------|------------|--------|
| **HuskyLens colors** | Train **Green** and **Purple** (Learn Multiple). Note which is ID1, which is ID2 → set `GREEN_HUSKY_COLOR_ID`, `PURPLE_HUSKY_COLOR_ID` in motif/spindexer config. | [Motif.java](src/main/java/org/firstinspires/ftc/teamcode/config/util/Motif.java) or Spindexer constants |
| **HuskyLens AprilTag** | Use TAG_RECOGNITION; confirm you see the game AprilTag and note its **ID** for each motif (e.g. 1=GPP, 2=PGP, 3=PPG). Set mapping in code. | [Motif.java](src/main/java/org/firstinspires/ftc/teamcode/config/util/Motif.java) |
| **HuskyLens position** | Ensure one spindexer slot passes under the lens; same lighting as match for color scan. | Mechanical |

---

## 3. Shooter (velocity compensation)

| Item | What to do | Where |
|------|------------|--------|
| **Hood angle range** | Measure min/max hood angle (degrees) → `HOOD_MIN_DEG`, `HOOD_MAX_DEG`. | [ShooterCalculatorVelocityComp.java](src/main/java/org/firstinspires/ftc/teamcode/config/util/ShooterCalculatorVelocityComp.java) |
| **Hood servo map** | At **min** and **max** angle, record servo positions → `HOOD_ANGLE_1_DEG`, `HOOD_SERVO_1`, `HOOD_ANGLE_2_DEG`, `HOOD_SERVO_2`. | Same file |
| **Flywheel ↔ launch speed** | At 2+ distances, measure actual launch speed (e.g. from range) and the flywheel velocity you used. Fit line: flywheelVel = slope * launchSpeedInPerSec + offset → `LAUNCH_SPEED_TO_VELOCITY_SLOPE`, `LAUNCH_SPEED_TO_VELOCITY_OFFSET`. | Same file |
| **Required angle at goal** | If you want the ball to enter at an angle (e.g. 0° = level), set `REQUIRED_ANGLE_AT_GOAL_DEG`. | Same file |

---

## 4. Pedro Pathing (autonomous)

| Item | What to do | Where |
|------|------------|--------|
| **Follower / drive** | Run Pedro tuners (forward, lateral, heading, etc.) and paste results into Constants. | [Constants.java](src/main/java/org/firstinspires/ftc/teamcode/config/pedro/Constants.java) |
| **Paths** | Build and tune paths for each auto; set start pose and waypoints. | Path files under `config/paths/` |

---

## 5. Pinpoint (odometry)

| Item | What to do | Where |
|------|------------|--------|
| **Reset point** | Pick a known field position (e.g. after crossing a line or under an AprilTag). In TeleOp, drive there and press a button to call `pinpoint.setPose(x, y, heading)` to correct drift. | TeleOp + [PinpointLocalizerSubsystem](src/main/java/org/firstinspires/ftc/teamcode/config/subsystem/PinpointLocalizerSubsystem.java) |
| **Initial pose** | In auto, set initial pose at start (e.g. 0,0,0 or your start tile). | Auto OpModes |

---

## 6. Quick reference – motor specs

| System | RPM | Notes |
|--------|-----|------|
| Spindexer | 435 | Used for INDEX_TIME_MS (46 ms per slot) |
| Shooter (each) | 6000 | Two flywheels; tune PIDF and launch-speed map |
| Intake | 1150 | Existing Intake subsystem |
| Chassis (each) | 435 | Pedro drive constants |

---

## 7. Motif (AprilTag → pattern)

| Item | What to do | Where |
|------|------------|--------|
| **AprilTag IDs** | Note which game AprilTag ID means which pattern (e.g. 1=GPP, 2=PGP, 3=PPG). Set mapping in `Motif.fromAprilTagId()`. | [Motif.java](src/main/java/org/firstinspires/ftc/teamcode/config/util/Motif.java) |
| **Green/Purple HuskyLens IDs** | After training colors on HuskyLens, set `GREEN_HUSKY_COLOR_ID` and `PURPLE_HUSKY_COLOR_ID` to match (1 or 2). | Same file |
| **Default motif** | If no tag is seen in time, `MotifScanHelper.DEFAULT_MOTIF` is used. | [MotifScanHelper.java](src/main/java/org/firstinspires/ftc/teamcode/config/util/MotifScanHelper.java) |

## 8. At start of each match

- **Auto:** In every auto that uses Robot_v2, run **MotifScanHelper.runMotifScan(r)** at the start (after creating Robot_v2). Turret goes to `TURRET_ANGLE_APRILTAG_SCAN_DEG`, HuskyLens reads AprilTag, motif is set on spindexer. Then run your paths. When feeding to shooter, use `r.spindexer.rotateToNextFeedSlot()` then feed one artifact, then `r.spindexer.advanceFeedPosition()` so you shoot in motif order (green/purple/purple etc.).
- **TeleOp:** Red and Blue Turret+Spindexer TeleOps already run the same motif scan in **init()**. Spindexer feed order follows motif for the whole match.

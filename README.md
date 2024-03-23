# CRESCENDO Robot Code
## Specs
- NEO V1.1 motors
- SparkMAX motor controllers
- SDS MK4i swerve modules
    - L2 gear ratio
    - CANCoder
## Subsystems
- Swerve drivebase
- Elevator
- Arm
- Shooter
- Intake
## Libraries
- YAGSL
- NavX
- PathPlannerLib
- Phoenix6
- REVLib
## PathPlanner Path Naming Scheme
"FROM - TO"
### Speaker
SS - Source Side
SM - Middle
SA - Amp Side
### Home
H1 - Note closest to the amp
H2 - Note in between amp and stage
H3 - Note in front of stage post
### Center
C1 - Center note on amp's side
C2 - Center note in between absolute center and C1
C3 - Center note in the center
C4 - Center note in between absolute center and C5
C5 - Center note on source's side
## PathPlanner Auto Naming Scheme
"STARTING POSITION - x note"
See the path naming scheme for starting position
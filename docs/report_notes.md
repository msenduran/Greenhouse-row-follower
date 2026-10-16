# Report Notes

Report file name: `student1ID_student2ID_student3ID.pdf` (upload to CANVAS).

## Deadlines

- 19 Oct 2026 (Mon) 23:59 – group registration form: https://forms.gle/Zdcu4XHFDggFLXVh9
- 8 Nov 2026 (Sun) 23:59 – report submission
- 9 Nov 2026 (Mon) 11:10, K-Z06 – presentation + live demo (8 min + 2 min Q&A), backup video required

## Requirements Checklist

### Robot
- [ ] Original design for a clearly defined task (no TurtleBot or other ready-made models)
- [ ] URDF/Xacro: chassis, ≥2 driven wheels, casters/extra wheels as needed
- [ ] Drive type chosen + matching Gazebo plugin; robot moves on `/cmd_vel`
- [ ] Gazebo world with obstacles
- [ ] Single launch file: world + spawn + RViz

### Sensors (≥ number of members + 1 different types)
- [ ] All sensors integrated in simulation
- [ ] Data readable via ROS 2 and visualized (RViz / image window / plot)
- [ ] Each member responsible for ≥1 sensor; the extra sensor explicitly assigned
- Note: joint states, ground-truth pose and drive-plugin odometry do NOT count

### Behavior
- [ ] ≥1 Python/C++ node subscribing to sensor data and publishing motion commands
- [ ] Input topics, message types, decision rule and output documented

### Repository
- [ ] Public (or instructor has access), all members are collaborators
- [ ] Each member commits their own work
- [ ] README with installation and run steps

## Report Contents
- [ ] Names, student numbers, GitHub link
- [ ] Task, original design, locomotion, component connections
- [ ] Sensor types, placement, topics, message types + evidence of reading/visualization
- [ ] Contribution table
- [ ] Packages and nodes, behavior node decision rule
- [ ] Simulation results (motion, sensor outputs, effect of sensors on behavior)

## Contribution Table

| Member | Sensor | Integration | Data reading | Visualization |
|--------|--------|-------------|--------------|---------------|
| TODO   | TODO   |             |              |               |

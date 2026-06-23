# Husarion ROSbot — Search and Locate Project

Autonomous mobile robot that searches a simulated arena, identifies a colour-coded target, and reports its coordinates. It was built in Webots using Python and a priority-based Finite State Machine.


## Features

- **Priority-based Finite State Machine (FSM)** — five states evaluated every timestep, with higher-priority states overriding lower ones
- **360° obstacle detection** via 2D LiDAR
- **Colour-based target identification** using RGB camera pixel filtering
- **Depth-fused coordinate estimation** combining depth camera and proximity sensors
- **Grid-based exploration** for systematic, non-random environment coverage
- **Stuck recovery** — automatic reversal and re-orientation when progress stalls
- **Decoy rejection** — requires simultaneous satisfaction of four conditions before declaring goal reached

---

## Finite State Machine

The robot operates through five prioritised states:

| Priority | State | Trigger | Behaviour |
|----------|-------|---------|-----------|
| 0 (Highest) | `GOAL_REACHED` | Target detected, red pixel count ≥ threshold, proximity sensor distance < 0.1 m, target aligned within 50 px of frame centre | Stop, print coordinates, spin indefinitely |
| 1 | `RECOVERY` | Stuck counter > 60 or < 5 cm moved in 100 timesteps | Reverse 30 steps, spin until path is clear + 90° turned |
| 2 | `PROXI_READINGS` | Obstacle within 0.35 m (target not centred) | Turn toward the more open side |
| 3 | `NAV_TARGET` | Red target visible, no higher state active | Steer left/right/forward toward target centroid |
| 4 (Lowest) | `EXPLORE` | Default — nothing else applies | Grid-based forward/turn exploration in 0.5 × 0.5 m cells |

https://github.com/user-attachments/assets/441217f4-77cc-47eb-99ff-5137e2102bf4

https://github.com/user-attachments/assets/f76687b6-e691-4d5b-aef2-f41ba40b79e8

*Above video showing the grid-based exploration in action. When reset to the start, the robot navigates toward unvisited cells rather than retracing its path.*

https://github.com/user-attachments/assets/b19d08a9-ad30-4f43-9792-76e2ee49c8a1

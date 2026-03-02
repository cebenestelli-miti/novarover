# Mock mode sections to add (do not remove anything from existing docs)

The docs in `docs/` are root-owned, so they couldn't be edited from here. Add the following as new sections only.

---

## 1. ROS_Mower_README.md

**Add after the "4️⃣ Architectural Principles" section, before the final paragraph ("This document defines...").**

```markdown
---

# 5️⃣ Mock mode (traceability / deployment safety)

A **mock base interface** is provided so the autonomy stack can run and be tested without the physical robot or Arduino. It publishes `/base/heartbeat`, `/odom/raw`, and optionally `/ultrasonic/ranges` (all clear ranges), and subscribes to `/cmd_vel` and `/safety/stop` so the rest of the system behaves as in deployment.

- **Traceability:** The same bringup launch runs in mock mode (development) and with the real base (deployment); only the `publish_ultrasonic` setting differs.
- **Deployment safety:** When running with real hardware, the real base provides `/ultrasonic/ranges`. To avoid duplicate or conflicting ultrasonic data, either do not run the mock base, or run bringup with ultrasonics disabled on the mock:
  - `ros2 launch mower_base bringup.launch.py publish_ultrasonic:=false`

With `publish_ultrasonic:=false`, the mock still publishes heartbeat and odom for testing; it does not publish `/ultrasonic/ranges`, so the real base (or no ultrasonics) can be used.

---
```

---

## 2. ROS_Mower_SYSTEM_ARCHITECTURE.md

**Add at the end of the file, after "6) Safety Behaviors".**

```markdown
---

## 7) Mock mode (traceability / deployment safety)

A **mock base interface** is provided so the autonomy stack can run and be tested without the physical robot or Arduino. It publishes `/base/heartbeat`, `/odom/raw`, and optionally `/ultrasonic/ranges` (all clear ranges), and subscribes to `/cmd_vel` and `/safety/stop` so the rest of the system behaves as in deployment.

- **Traceability:** The same bringup launch runs in mock mode (development) and with the real base (deployment); only the `publish_ultrasonic` setting differs.
- **Deployment safety:** When running with real hardware, the real base provides `/ultrasonic/ranges`. To avoid duplicate or conflicting ultrasonic data, either do not run the mock base, or run bringup with ultrasonics disabled on the mock:
  - `ros2 launch mower_base bringup.launch.py publish_ultrasonic:=false`

With `publish_ultrasonic:=false`, the mock still publishes heartbeat and odom for testing; it does not publish `/ultrasonic/ranges`, so the real base (or no ultrasonics) can be used.
```

---

## 3. ROS_Mower_AI_CODER_SPEC.md

**Add at the end of the file, after "7) AI Coder Prompt".**

```markdown
---

## 8) Mock mode (traceability / deployment safety)

A **mock base interface** is provided so the autonomy stack can run and be tested without the physical robot or Arduino. It publishes `/base/heartbeat`, `/odom/raw`, and optionally `/ultrasonic/ranges` (all clear ranges), and subscribes to `/cmd_vel` and `/safety/stop` so the rest of the system behaves as in deployment.

- **Traceability:** The same bringup launch runs in mock mode (development) and with the real base (deployment); only the `publish_ultrasonic` setting differs.
- **Deployment safety:** When running with real hardware, the real base provides `/ultrasonic/ranges`. To avoid duplicate or conflicting ultrasonic data, either do not run the mock base, or run bringup with ultrasonics disabled on the mock:
  - `ros2 launch mower_base bringup.launch.py publish_ultrasonic:=false`

With `publish_ultrasonic:=false`, the mock still publishes heartbeat and odom for testing; it does not publish `/ultrasonic/ranges`, so the real base (or no ultrasonics) can be used.
```

---

## Applying the changes

From the workspace root, after fixing ownership (optional):

```bash
sudo chown corye:corye docs/*.md
```

Then either paste each block above into the corresponding file at the indicated place, or append each block to the end of the corresponding file (for README, insert the new section *before* the final paragraph instead of at the very end).

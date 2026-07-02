# QueenHaeBin
autovehi/
│
├─ arduino/
│  └─ mega_motor_controller/
│     └─ mega_motor_controller.ino
│
├─ config/
│  ├─ config.py
│  ├─ vehicle_params.py
│  └─ sensor_params.py
│
├─ src/
│  ├─ measurement/
│  │  ├─ __init__.py
│  │  ├─ camera_sensor.py
│  │  ├─ lidar_sensor.py
│  │  ├─ ultrasonic_sensor.py
│  │  ├─ arduino_serial.py
│  │  └─ traffic_light_detector.py
│  │
│  ├─ perception/
│  │  ├─ __init__.py
│  │  ├─ obstacle_detector.py
│  │  ├─ ultrasonic_obstacle.py
│  │  ├─ lane_detector.py
│  │  └─ object_detector.py
│  │
│  ├─ estimation/
│  │  ├─ __init__.py
│  │  ├─ vehicle_state.py
│  │  ├─ kalman_filter.py
│  │  └─ sensor_fusion.py
│  │
│  ├─ localization/
│  │  ├─ __init__.py
│  │  ├─ odometry.py
│  │  ├─ lidar_localization.py
│  │  └─ map_manager.py
│  │
│  ├─ decision/
│  │  ├─ __init__.py
│  │  ├─ behavior_state_machine.py
│  │  └─ rule_based_decision.py
│  │
│  ├─ planning/
│  │  ├─ __init__.py
│  │  ├─ path_planner.py
│  │  ├─ motion_planner.py
│  │  └─ nonholonomic_planner.py
│  │
│  ├─ control/
│  │  ├─ __init__.py
│  │  ├─ manual_drive.py
│  │  ├─ drive_controller.py
│  │  ├─ steering_controller.py
│  │  └─ vehicle_controller.py
│  │
│  ├─ learning/
│  │  ├─ __init__.py
│  │  ├─ dataset_logger.py
│  │  ├─ train_model.py
│  │  ├─ model_inference.py
│  │  └─ models/
│  │
│  ├─ visualization/
│  │  ├─ __init__.py
│  │  ├─ camera_view.py
│  │  ├─ lidar_view.py
│  │  ├─ ultrasonic_view.py
│  │  └─ debug_dashboard.py
│  │
│  └─ utils/
│     ├─ __init__.py
│     ├─ geometry.py
│     ├─ timing.py
│     └─ logger.py
│
├─ scripts/
│  ├─ run_camera.py
│  ├─ run_lidar.py
│  ├─ run_ultrasonic.py
│  ├─ run_drive_live.py
│  ├─ run_traffic_light.py
│  └─ run_full_system.py
│
├─ data/
│  ├─ raw/
│  ├─ processed/
│  ├─ logs/
│  └─ models/
│
└─ requirements.txt
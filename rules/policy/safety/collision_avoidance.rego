# Safety-critical collision avoidance policies
# Precedence Level: SAFETY (cannot be overridden by any lower layer)

package safety.collision_avoidance

import rego.v1

# Safety-critical rule: minimum following distance
# Based on vehicle speed and reaction time requirements
min_following_distance_meters := speed_kmh * 0.8 + 10 if {
    speed_kmh := input.vehicle.current_speed_kmh
    speed_kmh > 0
}

min_following_distance_meters := 10 if {
    input.vehicle.current_speed_kmh <= 0
}

# Minimum lateral clearance from obstacles
min_lateral_clearance_meters := 2.0

# Time-to-collision thresholds by vehicle class
ttc_thresholds := {
    "ugv": 3.0,
    "sedan": 2.8,
    "truck": 3.5,
    "mining": 4.0,
    "military": 3.2
}

# Emergency stop required if TTC is below threshold
emergency_stop_required if {
    ttc_seconds := input.perception.time_to_collision
    ttc_threshold := ttc_thresholds[input.vehicle.class]
    
    ttc_seconds < ttc_threshold
    ttc_seconds > 0  # Only if collision is imminent
    input.vehicle.relative_speed > 0  # Only if approaching
}

# Emergency stop required for critical lateral clearance
emergency_stop_required if {
    clearance := input.perception.lateral_clearance_meters
    clearance < min_lateral_clearance_meters
    input.vehicle.current_speed_kmh > 5  # Above walking speed
}

# Speed reduction required for reduced visibility
speed_reduction_required := reduction_factor if {
    visibility_m := input.weather.visibility_meters
    current_speed := input.vehicle.current_speed_kmh
    
    visibility_m < 200
    visibility_m >= 100
    reduction_factor := 0.7  # 70% of current speed
}

speed_reduction_required := reduction_factor if {
    visibility_m := input.weather.visibility_meters
    current_speed := input.vehicle.current_speed_kmh
    
    visibility_m < 100
    visibility_m >= 50
    reduction_factor := 0.5  # 50% of current speed
}

# No operations allowed in severely reduced visibility
operations_prohibited if {
    visibility_m := input.weather.visibility_meters
    visibility_m < 50
}

# Following distance violation
following_distance_violation if {
    current_distance := input.perception.following_distance_meters
    required_distance := min_following_distance_meters
    
    current_distance < required_distance
}

# Safe following distance check
safe_following_distance if {
    current_distance := input.perception.following_distance_meters
    required_distance := min_following_distance_meters
    
    current_distance >= required_distance
}

# Lateral clearance violation
lateral_clearance_violation if {
    clearance := input.perception.lateral_clearance_meters
    clearance < min_lateral_clearance_meters
}

# Safe lane change permitted
safe_lane_change_permitted if {
    # Check target lane clearance
    target_clearance_front := input.perception.target_lane.front_clearance_meters
    target_clearance_rear := input.perception.target_lane.rear_clearance_meters
    
    # Require larger clearances for lane changes
    required_front := min_following_distance_meters * 1.5
    required_rear := min_following_distance_meters * 1.2
    
    target_clearance_front >= required_front
    target_clearance_rear >= required_rear
    
    # No emergency stop conditions
    not emergency_stop_required
}

# Intersection approach safety
intersection_approach_safe if {
    distance_to_intersection := input.perception.intersection_distance_meters
    current_speed := input.vehicle.current_speed_kmh
    
    # Can safely stop before intersection
    stopping_distance := (current_speed * current_speed) / (2 * 7.5) # Assuming 7.5 m/s² deceleration
    
    distance_to_intersection > stopping_distance * 1.3  # 30% safety margin
}

# Speed limits based on conditions
max_safe_speed_kmh := speed if {
    base_speed := input.odd.speed_limit_kmh
    
    # Apply weather-based reductions
    weather_factor := weather_speed_factor
    condition_factor := condition_speed_factor
    
    speed := base_speed * weather_factor * condition_factor
}

weather_speed_factor := factor if {
    visibility := input.weather.visibility_meters
    factor := 1.0 if visibility >= 500
    factor := 0.8 if visibility >= 200; visibility < 500
    factor := 0.6 if visibility >= 100; visibility < 200
    factor := 0.4 if visibility >= 50; visibility < 100
    factor := 0.0 if visibility < 50
}

condition_speed_factor := factor if {
    road_condition := input.road.condition
    factor := 1.0 if road_condition == "dry"
    factor := 0.9 if road_condition == "wet" 
    factor := 0.7 if road_condition == "snow"
    factor := 0.8 if road_condition == "dust"
    factor := 0.6 if road_condition == "ice"
}

# Overtaking safety check
overtaking_permitted if {
    # Sufficient visibility
    input.weather.visibility_meters >= 200
    
    # No oncoming traffic within safe distance
    oncoming_distance := input.perception.oncoming_vehicle_distance_meters
    oncoming_distance > 300  # 300m minimum
    
    # Target vehicle not too close
    target_distance := input.perception.following_distance_meters
    target_distance >= min_following_distance_meters
    
    # Road has passing lane or sufficient width
    road_width := input.road.width_meters
    road_width >= 6.0  # Minimum for safe overtaking
    
    # Not in restricted zone
    not input.location.overtaking_prohibited
}

# Construction zone safety
construction_zone_restrictions if {
    input.location.construction_zone
} then restrictions := {
    "max_speed_kmh": 30,
    "min_following_distance_m": min_following_distance_meters * 1.5,
    "lane_changes_prohibited": true,
    "increased_alertness": true
}

# School zone safety  
school_zone_active if {
    input.location.school_zone
    current_time := time.now_ns()
    
    # Active during school hours (assuming 7 AM - 4 PM on weekdays)
    weekday := time.weekday(current_time)
    hour := to_number(time.format([current_time])[0])
    
    weekday >= 1  # Monday
    weekday <= 5  # Friday  
    hour >= 7
    hour <= 16
}

school_zone_restrictions if {
    school_zone_active
} then restrictions := {
    "max_speed_kmh": 25,
    "min_following_distance_m": min_following_distance_meters * 1.2,
    "pedestrian_priority": true,
    "enhanced_monitoring": true
}

# Parking safety
safe_parking_permitted if {
    # Not blocking traffic
    not input.location.traffic_lane
    
    # Not in restricted zone
    not input.location.parking_prohibited
    
    # Sufficient clearance to moving traffic
    traffic_clearance := input.perception.traffic_clearance_meters
    traffic_clearance >= 3.0
    
    # Vehicle can exit safely
    exit_visibility := input.perception.exit_visibility_meters
    exit_visibility >= 50
}

# Decision output structure
decision := {
    "allowed": allowed,
    "restrictions": restrictions_applied,
    "emergency_actions": emergency_actions,
    "warnings": warnings,
    "precedence_level": "safety"
}

# Core decision logic
allowed if {
    not emergency_stop_required
    not operations_prohibited
    safe_following_distance
    not lateral_clearance_violation
}

restrictions_applied contains restriction if {
    construction_zone_restrictions
    restriction := construction_zone_restrictions
}

restrictions_applied contains restriction if {
    school_zone_restrictions
    restriction := school_zone_restrictions  
}

restrictions_applied contains "speed_reduction" if {
    speed_reduction_required
}

emergency_actions contains "emergency_stop" if {
    emergency_stop_required
}

warnings contains "following_too_close" if {
    following_distance_violation
}

warnings contains "lateral_clearance_low" if {
    lateral_clearance_violation
}

warnings contains "visibility_reduced" if {
    input.weather.visibility_meters < 200
}

# Audit and logging
audit_log := {
    "timestamp": time.now_ns(),
    "policy": "safety.collision_avoidance", 
    "version": "1.0.0",
    "input_hash": crypto.sha256(json.marshal(input)),
    "decision": decision,
    "vehicle_id": input.vehicle.id,
    "location": input.location
}

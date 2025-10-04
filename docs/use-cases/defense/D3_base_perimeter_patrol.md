# D3 — Base Perimeter Patrol & Intrusion Alert

## Basic Information

**ID:** D3  
**Name:** Base Perimeter Patrol & Intrusion Alert  
**Sector:** Defense  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Defense Sector Lead

## Use Case Definition

**Actors:**
- Primary: Base Security Unit, Security Operations Center (SOC)
- Supporting: UGV Patrol Vehicle, Sensor Network, Response Teams

**Trip Type:** PATROL_RUN

**ODD (Operational Design Domain):**
- Geographic: Base perimeter roads and patrol paths, security zones
- Environmental: All weather conditions with appropriate sensor modes, temperature range -10°C to 55°C
- Time: 24/7 operations with day/night sensor switching
- Communications: Secure base network with redundant channels
- Other: Integration with fixed sensor networks and security systems

**Trigger:**
Scheduled patrol cycles or security alert from other systems

**Nominal Flow:**
1. SOC creates patrol schedule with randomization factors for unpredictability
2. System assigns patrol routes with coverage optimization and timing variance
3. UGV executes patrol with multi-modal sensing (visual, thermal, radar, acoustic)
4. Continuous anomaly detection against baseline environmental models
5. Regular status updates to security systems with minimal alert threshold
6. Upon anomaly detection, system classifies threat level and confidence
7. High-confidence threats trigger immediate alert with evidence package
8. UGV maintains observation position or investigates based on SOC direction
9. System logs comprehensive patrol data for pattern analysis and reporting

**Variants / Edge Cases:**
- Intrusion detection: Classification and tracking protocol
- Environmental false positives: Filtering and verification procedures
- Patrol vehicle compromise: Failsafe and alert escalation
- Weather degradation: Sensor mode switching and threshold adjustments
- Multi-zone coordination: Handoff between patrol sectors and units

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Perimeter coverage: ≥ 95% of designated areas
  - False positive rate: ≤ 3% of alerts
  - Detection accuracy: ≥ 98% for human-sized objects at 100m
  - Alert response time: ≤ 60 seconds from detection to SOC notification
- **P1 (Should have):**
  - Pattern unpredictability index: ≥ 85% (measure of patrol randomization)
  - Battery/fuel efficiency: +30% vs. manned patrols
  - Multi-sensor fusion accuracy: ≥ 95% in adverse conditions
  - Evidence package completeness: 100% with all required data elements

**Dependencies:**
- **Services:**
  - `patrol-service`: Route planning and execution
  - `analytics`: Anomaly detection and pattern recognition
  - `v2x-service`: Integration with fixed sensor networks
  - `alerts-incident`: Security alert management
  - `map-service`: Detailed security zone mapping
- **Rules:**
  - `rules/odd/defense/patrol_rules.yaml`: Patrol behavior and coverage requirements
  - `rules/policy/security/anomaly_detection.yaml`: Threat classification thresholds
  - `rules/policy/security/evidence_collection.yaml`: Evidence gathering protocols
- **External Systems:**
  - Base Security System: Alert integration and response coordination
  - Video Management System: Evidence storage and retrieval
  - Access Control System: Integration for contextual awareness

**Risks & Mitigations:**
- **False positives overwhelming operators:**
  - Impact: High
  - Mitigation: Multi-stage verification, confidence scoring, contextual filtering
- **Sensor spoofing or jamming:**
  - Impact: High
  - Mitigation: Multi-modal sensing, anomaly detection in sensor data, anti-spoofing protocols
- **Patrol pattern predictability:**
  - Impact: Medium
  - Mitigation: AI-driven randomization, timing variance, coverage optimization
- **Weather impact on detection:**
  - Impact: Medium
  - Mitigation: Adaptive sensor modes, weather-specific thresholds, fusion algorithms

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `defense/patrol/intrusion_detection.json`: Various intrusion scenarios and approaches
  - `defense/patrol/environmental_challenges.json`: Weather and lighting conditions
  - `defense/patrol/false_positive_testing.json`: Common false trigger situations
- **Logs/Telemetry:**
  - Coverage metrics: patrol completeness, timing, randomization
  - Detection metrics: true/false positives/negatives, classification accuracy
  - System health: sensor status, communications integrity
- **Gates:**
  - Detection accuracy ≥ 98% in controlled tests
  - False positive rate ≤ 3% over 7-day continuous operation
  - Coverage verification with heat map analysis

**Rollout Plan:**
- **Phase 1:** Single patrol sector during daylight hours with human monitoring
- **Phase 2:** Multiple sectors with 24-hour operations
- **Phase 3:** Full perimeter integration with fixed sensor network

## Additional Information

**Related Use Cases:**
- D4: Route Clearance Recon
- D11: Runway FOD Patrol
- D24: LE/Force Interaction HMI

**References:**
- Base Security Operations Manual
- Perimeter Intrusion Detection Standards
- Multi-sensor Fusion Algorithms Documentation

**Notes:**
This use case represents a persistent security application with high reliability requirements. Success here demonstrates the system's ability to maintain vigilance over extended periods while minimizing false alarms and providing actionable intelligence.

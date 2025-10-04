# M25 — Autonomous Underground Mapping

## Basic Information

**ID:** M25  
**Name:** Autonomous Underground Mapping  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Mine Surveyors, Mine Planning
- Supporting: Geotechnical Engineers, Operations Management, Safety Department

**Trip Type:** UNDERGROUND_MAPPING_RUN

**ODD (Operational Design Domain):**
- Geographic: Underground mine workings, development headings, stopes, infrastructure
- Environmental: Underground conditions with variable lighting, temperature range 5°C to 40°C
- Time: Primarily during maintenance periods with limited production interference
- Communications: Underground mesh network with high-capacity local storage
- Other: Operation with specialized mapping equipment and survey-grade positioning systems

**Trigger:**
Scheduled survey update, development completion, or geotechnical assessment requirement

**Nominal Flow:**
1. System receives underground mapping mission with area details and accuracy requirements
2. Mapping strategy determination based on environment and precision needs
3. Vehicle is equipped with appropriate sensors (LiDAR, cameras, survey equipment)
4. Route planning incorporates optimal scanning positions and coverage patterns
5. Navigation to initial mapping location with precise positioning
6. Deployment of mapping equipment with calibration and reference point verification
7. Systematic data collection with multi-sensor integration and overlap verification
8. Real-time quality assessment with coverage verification and precision estimation
9. Continuous mapping with adaptive positioning for optimal data collection
10. Data processing with comprehensive 3D model generation and mine plan integration

**Variants / Edge Cases:**
- Difficult access areas: Alternative scanning positions and remote sensing techniques
- Poor lighting conditions: Active illumination and alternative sensing modes
- Water or mud: Adaptive routing and environmental protection measures
- Convergence monitoring: Precision comparison with historical data
- As-built verification: Design comparison and deviation reporting

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Mapping accuracy: ≤10mm precision for critical infrastructure
  - Coverage completeness: 100% of accessible areas mapped
  - Data quality: High-resolution point clouds with RGB overlay
  - Integration effectiveness: Seamless incorporation into mine planning systems
- **P1 (Should have):**
  - Mapping efficiency: +70% coverage speed vs. manual survey
  - Change detection: Accurate identification of geotechnical movement
  - Feature extraction: Automated identification of infrastructure and geological features
  - Visualization quality: Photorealistic 3D models with measurement capabilities

**Dependencies:**
- **Services:**
  - `underground-mapping-service`: Survey coordination and data processing
  - `routing`: Survey-optimized path planning
  - `analytics`: Point cloud analysis and feature extraction
  - `policy-engine`: Survey standards and data quality rules
  - `mine-planning-integration`: Model integration and comparison
- **Rules:**
  - `rules/odd/mining/underground_mapping.yaml`: Survey operation parameters
  - `rules/policy/mining/survey_standards.yaml`: Accuracy requirements
  - `rules/policy/mining/data_quality.yaml`: Collection and processing standards
  - `rules/policy/safety/underground_navigation.yaml`: Safe operation protocols
- **External Systems:**
  - Mine Planning System: Design data and as-built comparison
  - Geotechnical Monitoring System: Convergence and movement tracking
  - Survey Control Network: Reference points and coordinate systems

**Risks & Mitigations:**
- **Positioning errors:**
  - Impact: High
  - Mitigation: Survey control verification, redundant positioning, closed-loop traverses, reference point validation
- **Data gaps:**
  - Impact: Medium
  - Mitigation: Overlap verification, coverage planning, gap detection algorithms, complementary scanning positions
- **Environmental challenges:**
  - Impact: Medium
  - Mitigation: Multi-sensor approach, environmental calibration, adaptive sensing, equipment protection
- **Data volume management:**
  - Impact: Medium
  - Mitigation: Progressive data transfer, edge processing, compression techniques, prioritized transmission

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/mapping/standard_survey.json`: Routine mapping operations
  - `mining/mapping/precision_assessment.json`: Accuracy verification
  - `mining/mapping/change_detection.json`: Temporal comparison and movement identification
- **Logs/Telemetry:**
  - Mapping metrics: point density, accuracy estimation, coverage verification, feature extraction
  - Operational metrics: survey efficiency, positioning precision, equipment performance
  - Integration metrics: data transfer completeness, model generation quality, system incorporation
- **Gates:**
  - Survey team acceptance of data quality
  - Mine planning validation of model integration
  - Geotechnical verification of change detection capability

**Rollout Plan:**
- **Phase 1:** Basic mapping capability for main infrastructure
- **Phase 2:** Enhanced capability with comprehensive feature extraction and change detection
- **Phase 3:** Full integration with mine planning and geotechnical systems

## Additional Information

**Related Use Cases:**
- M17: Autonomous Slope Stability Monitoring
- M19: Autonomous Mine Rescue Support
- M24: Autonomous Ventilation Monitoring

**References:**
- Underground Survey Standards
- 3D Mapping Guidelines
- Geotechnical Monitoring Procedures

**Notes:**
This use case addresses the critical function of underground spatial data collection, which is essential for mine planning, safety assessment, and operational decision-making. Success here demonstrates the system's ability to create accurate and comprehensive 3D models while providing valuable insights for multiple stakeholders. The autonomous approach enables consistent and frequent mapping with reduced human exposure to challenging underground environments while significantly increasing coverage and efficiency.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of mapping requirements by mine area
- Measurable impact on planning efficiency
- Integration with existing mine systems

**Design:**
- Intuitive 3D visualization interfaces
- Clear data quality presentation
- Accessibility for various stakeholders

**Engineering:**
- Precision mapping technologies
- Underground positioning systems
- Data processing optimization

**Data:**
- Point cloud analytics and processing
- Feature extraction algorithms
- Change detection methodologies

**QA:**
- Accuracy validation procedures
- Coverage completeness verification
- System integration testing

**Security:**
- Mine data protection
- Access control protocols
- Version management security

**Operations:**
- Clear procedures for survey coordination
- Training for data utilization
- Performance monitoring protocols

**Mine Planning:**
- Model integration requirements
- As-built comparison methodology
- Design update procedures

**Geotechnical:**
- Movement detection thresholds
- Convergence monitoring protocols
- Risk assessment integration

# L8 — Empty Container Repositioning

## Basic Information

**ID:** L8  
**Name:** Empty Container Repositioning  
**Sector:** Logistics  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Logistics Sector Lead

## Use Case Definition

**Actors:**
- Primary: Container Pool Manager, Terminal Operations
- Supporting: Shipping Lines, Depot Operators, Trucking Companies

**Trip Type:** REPOSITIONING_RUN

**ODD (Operational Design Domain):**
- Geographic: Container terminals, depots, empty yards, and connecting corridors
- Environmental: Standard weather thresholds for container operations
- Time: 24/7 operations with off-peak preference
- Communications: Urban/industrial LTE/WiFi with high reliability
- Other: Operation with empty containers and specialized handling equipment

**Trigger:**
Imbalance thresholds, demand forecasts, or shipping line instructions

**Nominal Flow:**
1. System identifies container imbalances based on inventory levels and demand forecasts
2. Optimization algorithm generates repositioning plan to minimize empty moves
3. Autonomous vehicles are dispatched with prioritized container assignments
4. Vehicles navigate to source locations with precise terminal/depot routing
5. Empty container pickup is coordinated with yard equipment and verification systems
6. Containers are transported via optimal routes considering traffic and access restrictions
7. At destination, system coordinates precise positioning for container placement
8. Container handover is verified with digital documentation and status updates
9. System updates inventory management systems in real-time
10. Performance metrics are calculated for continuous optimization

**Variants / Edge Cases:**
- Multi-stop consolidation: Optimized pickup of multiple containers
- Equipment type constraints: Matching vehicle capabilities to container types
- Depot hour restrictions: Scheduling around operating windows
- Shipping line priority conflicts: Multi-tenant optimization
- Chassis management: Integration with chassis pools and availability

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Empty mile reduction: -25% vs. traditional operations
  - Repositioning cost: -20% per container move
  - On-time delivery: ≥95% within scheduled window
  - Documentation accuracy: 100% container tracking compliance
- **P1 (Should have):**
  - Multi-tenant optimization: ≥15% additional efficiency through pooling
  - Fuel/energy efficiency: +20% through route optimization
  - Yard congestion reduction: -30% peak queue time
  - Demand forecast accuracy: ≥85% for 7-day horizon

**Dependencies:**
- **Services:**
  - `dispatch`: Repositioning planning and scheduling
  - `routing`: Terminal and urban navigation
  - `analytics`: Imbalance detection and demand forecasting
  - `adapters/terminal`: Terminal Operating System integration
  - `adapters/depot`: Depot Management System integration
- **Rules:**
  - `rules/odd/logistics/container_operations.yaml`: Container handling parameters
  - `rules/policy/logistics/repositioning_priority.yaml`: Allocation and scheduling rules
  - `rules/policy/logistics/multi_tenant.yaml`: Shipping line agreements and constraints
  - `rules/policy/logistics/documentation.yaml`: Container tracking and verification
- **External Systems:**
  - Terminal Operating System: Container inventory and yard operations
  - Depot Management System: Empty container storage and processing
  - Shipping Line Systems: Container ownership and allocation rules

**Risks & Mitigations:**
- **Inventory discrepancies:**
  - Impact: High
  - Mitigation: Real-time verification, RFID/OCR validation, reconciliation processes
- **Demand forecast errors:**
  - Impact: Medium
  - Mitigation: Conservative buffers, rolling horizon updates, multi-scenario planning
- **Terminal congestion:**
  - Impact: High
  - Mitigation: Time slot booking, off-peak scheduling, alternative routing
- **Multi-tenant conflicts:**
  - Impact: Medium
  - Mitigation: Clear allocation rules, priority tiers, equitable optimization

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `logistics/empty/imbalance_optimization.json`: Various demand patterns
  - `logistics/empty/multi_tenant.json`: Complex allocation scenarios
  - `logistics/empty/terminal_integration.json`: TOS coordination workflows
- **Logs/Telemetry:**
  - Efficiency metrics: empty miles, container utilization, cost per move
  - Operational metrics: cycle time, terminal dwell, documentation compliance
  - Forecast metrics: demand prediction accuracy, inventory level stability
- **Gates:**
  - Empty mile reduction validation with operational data
  - Multi-tenant fairness verification with allocation audit
  - System integration verification with inventory reconciliation

**Rollout Plan:**
- **Phase 1:** Single terminal-depot corridor with one container type
- **Phase 2:** Multi-location network with varied container types
- **Phase 3:** Full multi-tenant optimization with demand forecasting

## Additional Information

**Related Use Cases:**
- L2: Container Terminal Berth Shuttle
- L7: Terminal Yard Optimization
- L9: Intermodal Rail Connection

**References:**
- Empty Container Management Best Practices
- Repositioning Optimization Methods
- Multi-tenant Container Pooling Standards

**Notes:**
This use case represents a significant efficiency opportunity in container logistics, addressing the persistent challenge of empty container imbalances. Success here demonstrates the system's ability to optimize across multiple stakeholders while reducing costs and environmental impact.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of repositioning optimization objectives
- Measurable impact on operational costs and efficiency
- Integration with existing container management workflows

**Design:**
- Intuitive visualization of imbalances and repositioning plans
- Clear interfaces for multi-tenant allocation and priorities
- Accessibility for terminal and depot operators

**Engineering:**
- Sophisticated optimization algorithms for multi-location balancing
- Robust integration with terminal and depot systems
- Reliable container tracking and verification

**Data:**
- Comprehensive container movement history and forecasting
- Multi-tenant allocation fairness metrics
- Efficiency analytics for continuous improvement

**QA:**
- Validation of optimization outcomes across varied demand scenarios
- Testing of terminal and depot system integrations
- Verification of container tracking accuracy

**Security:**
- Protection of multi-tenant commercial data
- Secure documentation and chain of custody
- Access controls for competitive information

**Operations:**
- Clear procedures for exception handling
- Training for terminal and depot staff
- Maintenance of container verification systems

**Commercial:**
- Multi-tenant agreement structures
- Cost allocation and billing integration
- Service level agreements for repositioning

**Sustainability:**
- Carbon footprint reduction tracking
- Empty mile elimination reporting
- Fuel/energy efficiency optimization

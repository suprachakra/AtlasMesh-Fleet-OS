# AtlasMesh Fleet OS - Simulation Gate Scenarios

This directory contains the simulation gate scenarios used in the CI/CD pipeline to validate the behavior of AtlasMesh Fleet OS across different sectors, vehicle types, and environmental conditions.

## Directory Structure

- `common/` - Common scenarios applicable across all sectors
- `mining/` - Mining-specific scenarios
- `defense/` - Defense-specific scenarios
- `logistics/` - Logistics-specific scenarios
- `ride-hail/` - Ride-hail-specific scenarios

## Scenario Format

Each scenario is defined using the OpenSCENARIO format with AtlasMesh-specific extensions. Scenarios include:

1. **Environment definition** - Map, weather, time of day
2. **Actor configuration** - Vehicles, pedestrians, obstacles
3. **Maneuvers** - Predefined movements and behaviors
4. **Success criteria** - Conditions that must be met for the scenario to pass

## CI/CD Integration

Scenarios are executed as part of the CI/CD pipeline using the following process:

1. Code changes trigger scenario execution
2. Scenarios are run in parallel across simulation environments
3. Results are collected and analyzed
4. Pass/fail status is reported back to the CI/CD pipeline

## Adding New Scenarios

To add a new scenario:

1. Create a new `.xosc` file in the appropriate sector directory
2. Define the scenario parameters
3. Add the scenario to the appropriate test suite in `sim/ci-gates/config/`
4. Add documentation to explain the purpose and expected behavior

## Scenario Categories

### Safety Scenarios

Safety scenarios validate the system's ability to handle safety-critical situations, including:

- Emergency braking
- Obstacle avoidance
- Edge case handling
- Fault injection

### Performance Scenarios

Performance scenarios validate the system's ability to meet performance requirements, including:

- Throughput
- Latency
- Resource utilization
- Scalability

### Functional Scenarios

Functional scenarios validate the system's ability to perform its intended functions, including:

- Mission execution
- Navigation
- Coordination
- Energy management

## Example Scenario

```xml
<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="0" date="2025-09-15" description="Example scenario"/>
  <ParameterDeclarations/>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="maps/mining/open_pit_01.xodr"/>
    <SceneGraphFile filepath="maps/mining/open_pit_01.osgb"/>
  </RoadNetwork>
  <Entities>
    <ScenarioObject name="ego">
      <Vehicle name="haul_truck_01" vehicleCategory="truck">
        <ParameterDeclarations/>
        <Performance maxSpeed="40" maxAcceleration="3" maxDeceleration="10"/>
        <BoundingBox>
          <Center x="0" y="0" z="0"/>
          <Dimensions width="3.5" length="15" height="4.5"/>
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="1.5" trackWidth="2.5" positionX="6" positionZ="0"/>
          <RearAxle maxSteering="0" wheelDiameter="1.5" trackWidth="2.5" positionX="-6" positionZ="0"/>
        </Axles>
        <Properties/>
      </Vehicle>
    </ScenarioObject>
    <ScenarioObject name="obstacle">
      <MiscObject name="rock_pile" miscObjectCategory="obstacle">
        <ParameterDeclarations/>
        <BoundingBox>
          <Center x="0" y="0" z="0"/>
          <Dimensions width="5" length="5" height="2"/>
        </BoundingBox>
        <Properties/>
      </MiscObject>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <Private entityRef="ego">
          <PrivateAction>
            <TeleportAction>
              <Position>
                <WorldPosition x="100" y="200" z="0" h="1.57" p="0" r="0"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
        <Private entityRef="obstacle">
          <PrivateAction>
            <TeleportAction>
              <Position>
                <WorldPosition x="150" y="200" z="0" h="0" p="0" r="0"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>
    <Story name="ObstacleAvoidanceStory">
      <Act name="ObstacleAvoidanceAct">
        <ManeuverGroup name="ObstacleAvoidanceSequence" maximumExecutionCount="1">
          <Actors selectTriggeringEntities="false">
            <EntityRef entityRef="ego"/>
          </Actors>
          <Maneuver name="ObstacleAvoidanceManeuver">
            <Event name="ApproachEvent" priority="overwrite">
              <Action name="ApproachAction">
                <PrivateAction>
                  <LongitudinalAction>
                    <SpeedAction>
                      <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
                      <SpeedActionTarget>
                        <AbsoluteTargetSpeed value="10"/>
                      </SpeedActionTarget>
                    </SpeedAction>
                  </LongitudinalAction>
                </PrivateAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition name="StartCondition" delay="0" conditionEdge="rising">
                    <ByValueCondition>
                      <SimulationTimeCondition value="0" rule="greaterThan"/>
                    </ByValueCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition name="ActStartCondition" delay="0" conditionEdge="rising">
              <ByValueCondition>
                <SimulationTimeCondition value="0" rule="greaterThan"/>
              </ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
      </Act>
    </Story>
    <StopTrigger>
      <ConditionGroup>
        <Condition name="StopCondition" delay="0" conditionEdge="rising">
          <ByEntityCondition>
            <TriggeringEntities triggeringEntitiesRule="any">
              <EntityRef entityRef="ego"/>
            </TriggeringEntities>
            <EntityCondition>
              <CollisionCondition>
                <EntityRef entityRef="obstacle"/>
              </CollisionCondition>
            </EntityCondition>
          </ByEntityCondition>
        </Condition>
      </ConditionGroup>
    </StopTrigger>
  </Storyboard>
</OpenSCENARIO>
```

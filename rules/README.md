# Policy Rules Engine

Policy-as-code system for the AtlasMesh Fleet OS with strict precedence hierarchy.

## Design Principles

- **Policy Precedence**: Safety > Law > Tenant > Sector > City > Fleet > Experiment
- **Declarative Rules**: All policies written in Rego (Open Policy Agent)
- **Versioning**: All policies are versioned and auditable
- **Testing**: Policy changes require comprehensive test coverage

## Directory Structure

```
rules/
├── policy/                 # Policy rules in Rego
│   ├── safety/            # Safety-critical policies (highest precedence)
│   ├── regulatory/        # Legal/regulatory compliance
│   ├── tenant/            # Customer-specific policies
│   ├── sector/            # Sector-specific rules
│   ├── city/              # City/regional policies
│   ├── fleet/             # Fleet-wide defaults
│   └── experimental/      # A/B test policies (lowest precedence)
├── data/                  # Policy data files
│   ├── geofences/
│   ├── speed_limits/
│   └── restricted_zones/
├── tests/                 # Policy unit tests
└── schema/                # Policy schema definitions
```

## Policy Precedence Hierarchy

1. **Safety** - Cannot be overridden by any lower layer
2. **Regulatory** - Jurisdiction-specific legal requirements
3. **Tenant** - Customer contractual obligations
4. **Sector** - Industry-specific requirements
5. **City** - Regional operational policies  
6. **Fleet** - Default operational parameters
7. **Experimental** - A/B test variants (only if all above allow)

## Example Policies

### Safety Policy (Highest Precedence)

```rego
# rules/policy/safety/collision_avoidance.rego
package safety.collision_avoidance

# Safety-critical rule: minimum following distance
min_following_distance_meters := 30

# Cannot be overridden by any lower precedence policy
allow_reduced_following_distance := false

# Emergency stop required if TTC < threshold
emergency_stop_required {
    input.ttc_seconds < 2.5
    input.relative_speed > 0
}
```

### Sector Policy

```rego
# rules/policy/sector/defense.rego
package sector.defense

# Defense-specific convoy spacing
convoy_spacing_meters := 100

# GPS-denied operation protocols
gps_denied_protocols {
    input.gps_quality < 3
    input.sector == "defense"
}

# Encrypted communications required
require_encrypted_comms := true
```

### City Policy

```rego
# rules/policy/city/abu_dhabi.rego
package city.abu_dhabi

# Speed limits for Abu Dhabi
max_speed_kmh := {
    "urban": 50,
    "highway": 80,
    "construction_zone": 30
}

# Dust storm protocols
dust_storm_restrictions {
    input.weather.visibility_meters < 200
    input.weather.dust_concentration > 1000  # µg/m³
}
```

## Policy Resolution

Policies are evaluated in precedence order. Conflicts are resolved by higher precedence winning:

```typescript
// Policy engine resolves conflicts automatically
const policyDecision = await policyEngine.evaluate({
  input: {
    sector: 'defense',
    city: 'abu_dhabi',
    vehicle: 'ugv_themis',
    current_speed: 45,
    weather: { visibility: 150 },
    ttc_seconds: 5.2
  }
});

// Result includes which policies applied and why
console.log(policyDecision.allowed); // true/false
console.log(policyDecision.reasons); // Array of policy evaluations
console.log(policyDecision.precedence); // Which precedence level made final decision
```

## ODD (Operational Design Domain) Policies

```rego
# rules/policy/odd/weather_constraints.rego
package odd.weather_constraints

# Define when operations must be restricted
operation_restricted {
    input.weather.temperature_celsius > 55
}

operation_restricted {
    input.weather.wind_speed_kmh > 60
    input.sector == "ride_hail"
}

# Degraded operation conditions
degraded_operation {
    input.weather.visibility_meters < 500
    input.weather.visibility_meters >= 200
}
```

## Geofencing Policies

```rego
# rules/policy/geofences/restricted_zones.rego
package geofences.restricted_zones

import data.geofences.military_zones
import data.geofences.airport_zones

# Check if location is in restricted area
in_restricted_zone {
    point_in_polygon(input.location, military_zones[_])
}

in_restricted_zone {
    point_in_polygon(input.location, airport_zones[_])
    input.sector != "defense"  # Defense vehicles may have clearance
}
```

## Policy Testing

All policies must have comprehensive unit tests:

```rego
# rules/tests/safety_test.rego
package safety.collision_avoidance

test_emergency_stop_required {
    emergency_stop_required with input as {
        "ttc_seconds": 2.0,
        "relative_speed": 10
    }
}

test_normal_following_allowed {
    not emergency_stop_required with input as {
        "ttc_seconds": 5.0,
        "relative_speed": 5
    }
}
```

## Policy Deployment

Policies are deployed through the policy service with validation:

```bash
# Validate all policies
opa test rules/

# Deploy to staging
kubectl apply -f deploy/policy-service/staging/

# Canary deployment for policy changes
./tools/policy/canary_deploy.sh rules/policy/city/abu_dhabi.rego
```

## Audit and Compliance

All policy decisions are logged for audit:

```json
{
  "timestamp": "2025-09-13T10:30:00Z",
  "trip_id": "trip_12345",
  "vehicle_id": "ugv_001",
  "policy_decision": {
    "allowed": true,
    "policies_evaluated": [
      "safety.collision_avoidance",
      "sector.defense",
      "city.abu_dhabi"
    ],
    "final_decision_precedence": "safety",
    "reason": "All safety constraints satisfied"
  }
}
```

## Integration with Services

Services query the policy engine before making decisions:

```typescript
// In dispatch-service
const canDispatch = await policyEngine.evaluate({
  input: {
    trip: tripRequest,
    vehicle: vehicleStatus,
    weather: currentWeather,
    location: startLocation
  }
});

if (!canDispatch.allowed) {
  throw new PolicyViolationError(canDispatch.reasons);
}
```

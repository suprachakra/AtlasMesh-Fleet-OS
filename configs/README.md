# Configuration Management

Vehicle-agnostic, platform-agnostic, and sector-agnostic configuration system.

## Design Principles

- **Separation of Concerns**: Configuration ≠ Code ≠ Policy
- **Composability**: Base configs + overlays for specialization
- **Versioning**: All configs are versioned and immutable
- **Validation**: Schema validation with compatibility testing

## Directory Structure

```
configs/
├── base/                    # Base configurations
│   ├── schema/             # JSON Schema definitions
│   ├── slos.yaml           # Service Level Objectives
│   └── thermal_derating.yaml
├── sectors/                # Sector-specific configurations
│   ├── defense/
│   ├── mining/
│   ├── logistics/
│   └── ride_hail/
├── vehicles/               # Vehicle capability manifests
│   ├── ugv_themis/
│   ├── haul_truck_cat/
│   └── sedan_generic/
├── cities/                 # City/region specific configs
│   ├── abu_dhabi/
│   ├── riyadh/
│   └── doha/
└── tenants/               # Tenant/customer overrides
    ├── tenant_a/
    └── tenant_b/
```

## Configuration Hierarchy

Configurations are applied in order of precedence:

1. **Base** - Universal defaults
2. **Sector** - Sector-specific settings (defense, mining, logistics, ride-hail)
3. **Vehicle** - Vehicle type capabilities and limits
4. **City** - Regional/jurisdictional requirements
5. **Tenant** - Customer-specific overrides

Higher precedence overrides lower precedence for conflicting settings.

## Key Configuration Types

### Vehicle Capability Manifest

Defines what each vehicle type can and cannot do:

```yaml
# vehicles/ugv_themis/capability_manifest.yaml
vehicle_id: "ugv_themis"
vehicle_class: "ugv"
max_speed: 25  # km/h
max_payload: 1000  # kg
drive_by_wire:
  steering: true
  throttle: true
  brake: true
  gear: true
sensors:
  - type: "lidar"
    model: "velodyne_vlp16"
  - type: "camera"
    count: 6
compute_class: "nvidia_jetson_agx_orin"
power_budget: 2000  # watts
thermal_limits:
  operating_temp: [-20, 55]  # celsius
  storage_temp: [-40, 70]
```

### Sector Configuration

Defines operational parameters per sector:

```yaml
# sectors/defense/config.yaml
sector: "defense"
priority_model: "mission_first"
safety_margins:
  following_distance: 50  # meters
  convoy_spacing: 100
kpis:
  assist_budget_per_100_trips: 1.0
  max_eta_variance: 0.15
special_requirements:
  gps_denied_operation: true
  encrypted_communications: true
  operational_security: "classified"
```

### City/ODD Configuration

Defines regional constraints and requirements:

```yaml
# cities/abu_dhabi/config.yaml
city: "abu_dhabi"
country: "uae"
timezone: "Asia/Dubai"
weather_conditions:
  max_operating_temp: 55
  dust_storms: true
  sandstorm_protocols: true
regulations:
  data_residency: "uae"
  max_speed_urban: 50
  autonomous_permits_required: true
map_providers:
  - "here_maps"
  - "local_survey_data"
```

## Schema Validation

All configurations are validated against JSON schemas:

```bash
# Validate all configurations
npm run validate:schemas

# Validate compatibility matrix
npm run validate:compat
```

## Usage in Services

Services access configuration through the Configuration Service:

```typescript
import { ConfigService } from '@atlasmesh/config-service';

// Get resolved config for specific context
const config = await configService.resolve({
  sector: 'mining',
  vehicle: 'haul_truck_cat',
  city: 'riyadh',
  tenant: 'mining_corp_a'
});
```

## Environment-Specific Overlays

Use Kustomize for environment-specific modifications:

```yaml
# deploy/kustomize/overlays/staging/kustomization.yaml
resources:
- ../../base

patchesStrategicMerge:
- config-overrides.yaml
```

## Testing

Compatibility testing ensures no config combination breaks the system:

```bash
# Run compatibility matrix tests
make test-compat

# Test specific combination
npm run test:compat -- --sector=defense --vehicle=ugv_themis --city=abu_dhabi
```

# AtlasMesh Fleet OS - Map Configuration and Examples

This directory contains map configuration files, examples, and validation tools for AtlasMesh Fleet OS.

## Directory Structure

- `examples/` - Example map files in supported formats
  - `lanelet2/` - Lanelet2 format examples (.osm)
  - `opendrive/` - OpenDRIVE format examples (.xodr)
- `validators/` - Map validation tools and scripts

## Supported Map Formats

### Lanelet2

Lanelet2 is the primary operational format used internally by AtlasMesh Fleet OS. It is particularly well-suited for autonomous driving applications due to its rich semantic features and regulatory elements.

Key features:
- Lane-level representation with explicit left and right boundaries
- Support for traffic rules and regulatory elements
- Hierarchical structure with relations between elements
- Efficient querying for routing and planning

Example usage:
```bash
python configs/map/validators/validate_map.py --format lanelet2 --file configs/map/examples/lanelet2/intersection_example.osm
```

### OpenDRIVE

OpenDRIVE is used as an exchange format for interoperability with other systems and simulation environments. AtlasMesh Fleet OS includes converters between Lanelet2 and OpenDRIVE.

Key features:
- Detailed road network representation
- Support for complex road geometries
- Integration with simulation environments like CARLA
- Industry standard for road network exchange

Example usage:
```bash
python configs/map/validators/validate_map.py --format opendrive --file configs/map/examples/opendrive/straight_road_example.xodr
```

## Map Validation

The `validators/` directory contains tools for validating map files to ensure they meet AtlasMesh Fleet OS requirements:

- `validate_map.py` - Validates Lanelet2 and OpenDRIVE maps for correctness and compliance

### Validation Criteria

Maps are validated against the following criteria:

1. **Structural validity** - Correct XML structure, required elements present
2. **Semantic validity** - Logical relationships between elements
3. **AtlasMesh requirements** - Additional requirements specific to AtlasMesh Fleet OS
4. **Regulatory elements** - Traffic rules, speed limits, etc.

## Map Conversion

AtlasMesh Fleet OS includes tools for converting between map formats:

```bash
# Convert from Lanelet2 to OpenDRIVE
python tools/map-converter/convert_map.py --from lanelet2 --to opendrive --input configs/map/examples/lanelet2/intersection_example.osm --output converted_map.xodr

# Convert from OpenDRIVE to Lanelet2
python tools/map-converter/convert_map.py --from opendrive --to lanelet2 --input configs/map/examples/opendrive/straight_road_example.xodr --output converted_map.osm
```

## Map Integration

Maps are integrated into AtlasMesh Fleet OS through the Map Service, which:

1. Loads maps from files or map providers
2. Validates maps against requirements
3. Converts between formats as needed
4. Provides map data to other services
5. Tracks map provenance and versioning

## References

- [Lanelet2 Documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [OpenDRIVE Specification](https://www.asam.net/standards/detail/opendrive/)
- AtlasMesh Map-Source-Agnostic ADR: `docs/ADR/0005-map-source-agnostic.md`

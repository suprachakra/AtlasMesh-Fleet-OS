# HD Map Service

> **TL;DR:** High-definition mapping service providing precise map data, lane-level navigation, and real-time map updates for autonomous vehicle operations.

## Overview

The HD Map Service provides high-definition mapping capabilities for autonomous vehicles, including lane-level precision, traffic sign recognition, and real-time map updates. It serves as the foundation for precise localization and path planning.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    HD MAP SERVICE                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐     │
│  │ Map Storage  │  │ Map Builder  │  │ Map Server  │     │
│  │   Engine     │  │   Engine     │  │   Engine    │     │
│  └──────────────┘  └──────────────┘  └─────────────┘     │
│         ▲                 ▲                  │             │
│         │                 │                  ▼             │
│  ┌──────────────────────────────────────────────────┐     │
│  │         Map Database (PostgreSQL + S3)          │     │
│  └──────────────────────────────────────────────────┘     │
│         ▲                 ▲                  │             │
│  ┌──────┴─────┐    ┌──────┴──────┐   ┌──────┴──────┐    │
│  │   Map      │    │   Map       │   │   Map       │    │
│  │  Ingestion │    │  Processing │   │  Serving    │    │
│  └────────────┘    └─────────────┘   └─────────────┘    │
│         ▲                 ▲                  │             │
└─────────┼─────────────────┼──────────────────┼─────────────┘
          │                 │                  │
    ┌─────┴────┐      ┌─────┴─────┐     ┌─────┴─────┐
    │ Vehicle  │      │  Survey   │     │   Cloud   │
    │  Data    │      │  Data     │     │  Updates  │
    └──────────┘      └───────────┘     └───────────┘
```

## Features

- **HD Map Storage**: High-precision map data with lane-level accuracy
- **Map Building**: Automated map construction from survey data
- **Map Serving**: Real-time map data delivery to vehicles
- **Lane Detection**: Precise lane geometry and connectivity
- **Traffic Signs**: Traffic sign recognition and positioning
- **Road Network**: Complete road network topology
- **Map Updates**: Real-time map updates and versioning
- **Geospatial Queries**: Efficient spatial data queries
- **Map Validation**: Data quality and consistency checking

## API Endpoints

### Map Data
- `GET /api/v1/hd-map/tiles/{z}/{x}/{y}` - Get map tile
- `GET /api/v1/hd-map/region` - Get map data for region
- `GET /api/v1/hd-map/lanes` - Get lane data
- `GET /api/v1/hd-map/traffic-signs` - Get traffic sign data
- `GET /api/v1/hd-map/road-network` - Get road network data

### Map Management
- `POST /api/v1/hd-map/upload` - Upload map data
- `PUT /api/v1/hd-map/update` - Update map data
- `DELETE /api/v1/hd-map/delete` - Delete map data
- `GET /api/v1/hd-map/versions` - Get map versions
- `POST /api/v1/hd-map/validate` - Validate map data

### Geospatial Queries
- `POST /api/v1/hd-map/query/nearby` - Find nearby features
- `POST /api/v1/hd-map/query/route` - Get route geometry
- `POST /api/v1/hd-map/query/intersection` - Get intersection data
- `POST /api/v1/hd-map/query/lane-change` - Get lane change options

## Database Schema

### Map Tiles
- `map_tiles` - Map tile data with geometry
- `tile_metadata` - Tile metadata and properties
- `tile_versions` - Tile versioning information

### Lane Data
- `lanes` - Lane geometry and properties
- `lane_connections` - Lane connectivity relationships
- `lane_markings` - Lane marking data
- `lane_restrictions` - Lane usage restrictions

### Traffic Signs
- `traffic_signs` - Traffic sign data
- `sign_geometry` - Sign geometry and positioning
- `sign_regulations` - Sign regulation data

### Road Network
- `roads` - Road segment data
- `intersections` - Intersection geometry
- `road_attributes` - Road properties and attributes

## Dependencies

- **PostgreSQL**: Primary database for map data
- **PostGIS**: Geospatial extensions
- **S3/MinIO**: Object storage for large map files
- **Redis**: Caching layer
- **GDAL**: Geospatial data processing
- **OpenStreetMap**: Base map data
- **Mapbox**: Map visualization

## Deployment

### Docker
```bash
docker build -t hd-map-service .
docker run --rm -it -p 8080:8080 hd-map-service
```

### Kubernetes
```bash
kubectl apply -f k8s/
```

### Local Development
```bash
# Install dependencies
go mod download

# Run the service
go run ./cmd/main.go
```

## Performance

- **Tile Serving**: < 50ms response time
- **Spatial Queries**: < 100ms for 1km radius
- **Map Updates**: Real-time processing
- **Storage**: Petabyte-scale map data
- **Concurrency**: 1000+ concurrent requests

## Monitoring

- **Performance Metrics**: Response times, throughput
- **Storage Metrics**: Disk usage, data growth
- **Quality Metrics**: Map accuracy, completeness
- **Usage Metrics**: API calls, data access patterns

## Troubleshooting

### Common Issues
1. **Slow Queries**: Check database indexes and query optimization
2. **Memory Usage**: Monitor tile caching and memory limits
3. **Storage Issues**: Check disk space and S3 connectivity
4. **Data Quality**: Validate map data consistency

### Logs
- **Application Logs**: `/opt/atlasmesh/logs/hd-map-service/`
- **Database Logs**: PostgreSQL logs
- **Storage Logs**: S3/MinIO logs

## Development

### Adding New Map Features
1. Define data schema
2. Implement processing logic
3. Add API endpoints
4. Update documentation
5. Add tests

### Map Data Sources
1. **Survey Data**: High-precision GPS surveys
2. **Vehicle Data**: Crowdsourced map updates
3. **Satellite Imagery**: Aerial and satellite data
4. **OpenStreetMap**: Community map data

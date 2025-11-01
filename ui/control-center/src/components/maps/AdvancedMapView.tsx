import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Box, Paper, Typography, Switch, FormControlLabel, Slider, Chip, IconButton, Tooltip } from '@mui/material';
import { styled } from '@mui/material/styles';
import { 
  Layers as LayersIcon,
  Traffic as TrafficIcon,
  Construction as ConstructionIcon,
  Sensors as SensorsIcon,
  ThreeDRotation as ThreeDIcon,
  Visibility as VisibilityIcon,
  VisibilityOff as VisibilityOffIcon,
  Settings as SettingsIcon,
  Refresh as RefreshIcon,
  Fullscreen as FullscreenIcon,
  FilterAlt as FilterIcon
} from '@mui/icons-material';
import mapboxgl from 'mapbox-gl';
import 'mapbox-gl/dist/mapbox-gl.css';

// Set Mapbox access token
mapboxgl.accessToken = process.env.REACT_APP_MAPBOX_TOKEN || 'pk.eyJ1IjoiYXRsYXNtZXNoIiwiYSI6ImNscXh5ejB4eTBhbmkya3BjdGVzZXR6YjEifQ.example';

interface MapLayer {
  id: string;
  name: string;
  type: 'traffic' | 'construction' | 'sensors' | 'weather' | 'routes' | 'geofences' | 'heatmap';
  visible: boolean;
  opacity: number;
  color: string;
  icon: React.ReactNode;
  data?: any[];
}

interface SensorOverlay {
  id: string;
  type: 'camera' | 'lidar' | 'radar' | 'weather' | 'air_quality' | 'noise';
  position: [number, number];
  status: 'online' | 'offline' | 'maintenance';
  data: {
    value: number;
    unit: string;
    timestamp: Date;
    quality: 'good' | 'fair' | 'poor';
  };
  coverage?: {
    radius: number;
    angle?: number;
    direction?: number;
  };
}

interface TrafficData {
  segmentId: string;
  coordinates: [number, number][];
  congestionLevel: 'free' | 'moderate' | 'heavy' | 'severe';
  speed: number;
  averageSpeed: number;
  incidents: TrafficIncident[];
}

interface TrafficIncident {
  id: string;
  type: 'accident' | 'construction' | 'closure' | 'weather' | 'event';
  severity: 'low' | 'medium' | 'high' | 'critical';
  position: [number, number];
  description: string;
  startTime: Date;
  estimatedDuration?: number;
  affectedLanes?: number;
}

interface ConstructionZone {
  id: string;
  name: string;
  type: 'road_work' | 'bridge_work' | 'utility' | 'maintenance';
  status: 'planned' | 'active' | 'completed' | 'suspended';
  geometry: {
    type: 'Polygon' | 'LineString';
    coordinates: number[][];
  };
  startDate: Date;
  endDate: Date;
  impact: {
    laneClosures: number;
    speedReduction: number;
    detourRequired: boolean;
  };
  contractor: string;
  permit: string;
}

const MapContainer = styled(Box)(({ theme }) => ({
  position: 'relative',
  width: '100%',
  height: '600px',
  borderRadius: theme.shape.borderRadius,
  overflow: 'hidden',
  '& .mapboxgl-map': {
    borderRadius: theme.shape.borderRadius,
  },
}));

const LayerControls = styled(Paper)(({ theme }) => ({
  position: 'absolute',
  top: theme.spacing(2),
  right: theme.spacing(2),
  padding: theme.spacing(2),
  minWidth: 300,
  maxHeight: '80%',
  overflowY: 'auto',
  zIndex: 1000,
  backgroundColor: 'rgba(255, 255, 255, 0.95)',
  backdropFilter: 'blur(10px)',
}));

const MapControls = styled(Box)(({ theme }) => ({
  position: 'absolute',
  top: theme.spacing(2),
  left: theme.spacing(2),
  display: 'flex',
  flexDirection: 'column',
  gap: theme.spacing(1),
  zIndex: 1000,
}));

const SensorPopup = styled(Paper)(({ theme }) => ({
  padding: theme.spacing(2),
  minWidth: 200,
  maxWidth: 300,
}));

const AdvancedMapView: React.FC = () => {
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<mapboxgl.Map | null>(null);
  const [mapLoaded, setMapLoaded] = useState(false);
  const [is3DEnabled, setIs3DEnabled] = useState(false);
  const [showLayerControls, setShowLayerControls] = useState(true);
  const [mapStyle, setMapStyle] = useState<'streets' | 'satellite' | 'hybrid'>('streets');
  
  const [layers, setLayers] = useState<MapLayer[]>([
    {
      id: 'traffic',
      name: 'Traffic Flow',
      type: 'traffic',
      visible: true,
      opacity: 0.8,
      color: '#FF5722',
      icon: <TrafficIcon />,
    },
    {
      id: 'construction',
      name: 'Construction Zones',
      type: 'construction',
      visible: true,
      opacity: 0.7,
      color: '#FF9800',
      icon: <ConstructionIcon />,
    },
    {
      id: 'sensors',
      name: 'Sensor Network',
      type: 'sensors',
      visible: true,
      opacity: 0.9,
      color: '#2196F3',
      icon: <SensorsIcon />,
    },
    {
      id: 'weather',
      name: 'Weather Overlay',
      type: 'weather',
      visible: false,
      opacity: 0.6,
      color: '#4CAF50',
      icon: <LayersIcon />,
    },
    {
      id: 'routes',
      name: 'Optimized Routes',
      type: 'routes',
      visible: true,
      opacity: 0.8,
      color: '#9C27B0',
      icon: <LayersIcon />,
    },
  ]);

  const [sensorData, setSensorData] = useState<SensorOverlay[]>([]);
  const [trafficData, setTrafficData] = useState<TrafficData[]>([]);
  const [constructionZones, setConstructionZones] = useState<ConstructionZone[]>([]);

  // Initialize map
  useEffect(() => {
    if (!mapContainer.current || map.current) return;

    map.current = new mapboxgl.Map({
      container: mapContainer.current,
      style: 'mapbox://styles/mapbox/streets-v12',
      center: [55.2708, 25.2048], // Dubai coordinates
      zoom: 12,
      pitch: 0,
      bearing: 0,
      antialias: true,
    });

    map.current.on('load', () => {
      setMapLoaded(true);
      initializeMapLayers();
      loadMapData();
    });

    // Add navigation controls
    map.current.addControl(new mapboxgl.NavigationControl(), 'bottom-left');
    map.current.addControl(new mapboxgl.FullscreenControl(), 'bottom-left');

    return () => {
      if (map.current) {
        map.current.remove();
        map.current = null;
      }
    };
  }, []);

  const initializeMapLayers = useCallback(() => {
    if (!map.current || !mapLoaded) return;

    // Add 3D buildings layer
    if (!map.current.getLayer('3d-buildings')) {
      map.current.addLayer({
        id: '3d-buildings',
        source: 'composite',
        'source-layer': 'building',
        filter: ['==', 'extrude', 'true'],
        type: 'fill-extrusion',
        minzoom: 15,
        paint: {
          'fill-extrusion-color': '#aaa',
          'fill-extrusion-height': [
            'interpolate',
            ['linear'],
            ['zoom'],
            15,
            0,
            15.05,
            ['get', 'height']
          ],
          'fill-extrusion-base': [
            'interpolate',
            ['linear'],
            ['zoom'],
            15,
            0,
            15.05,
            ['get', 'min_height']
          ],
          'fill-extrusion-opacity': 0.6
        }
      });
    }

    // Add traffic layer source
    if (!map.current.getSource('traffic-data')) {
      map.current.addSource('traffic-data', {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: []
        }
      });
    }

    // Add construction zones source
    if (!map.current.getSource('construction-zones')) {
      map.current.addSource('construction-zones', {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: []
        }
      });
    }

    // Add sensors source
    if (!map.current.getSource('sensors')) {
      map.current.addSource('sensors', {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: []
        }
      });
    }
  }, [mapLoaded]);

  const loadMapData = useCallback(async () => {
    try {
      // Load sensor data
      const sensorsResponse = await fetch('/api/v1/sensors/locations');
      const sensors = await sensorsResponse.json();
      setSensorData(sensors);

      // Load traffic data
      const trafficResponse = await fetch('/api/v1/traffic/current');
      const traffic = await trafficResponse.json();
      setTrafficData(traffic);

      // Load construction zones
      const constructionResponse = await fetch('/api/v1/construction/zones');
      const construction = await constructionResponse.json();
      setConstructionZones(construction);

      // Update map layers with data
      updateMapLayers();
    } catch (error) {
      console.error('Failed to load map data:', error);
      // Load mock data
      loadMockData();
    }
  }, []);

  const loadMockData = () => {
    // Mock sensor data
    setSensorData([
      {
        id: 'cam-001',
        type: 'camera',
        position: [55.2708, 25.2048],
        status: 'online',
        data: { value: 95, unit: '%', timestamp: new Date(), quality: 'good' },
        coverage: { radius: 100, angle: 60, direction: 45 }
      },
      {
        id: 'lidar-001',
        type: 'lidar',
        position: [55.2800, 25.2100],
        status: 'online',
        data: { value: 87, unit: '%', timestamp: new Date(), quality: 'good' },
        coverage: { radius: 200 }
      },
      {
        id: 'weather-001',
        type: 'weather',
        position: [55.2650, 25.2000],
        status: 'online',
        data: { value: 32, unit: '°C', timestamp: new Date(), quality: 'good' }
      }
    ]);

    // Mock traffic data
    setTrafficData([
      {
        segmentId: 'seg-001',
        coordinates: [[55.2700, 25.2040], [55.2750, 25.2060]],
        congestionLevel: 'moderate',
        speed: 45,
        averageSpeed: 60,
        incidents: []
      }
    ]);

    // Mock construction zones
    setConstructionZones([
      {
        id: 'const-001',
        name: 'Sheikh Zayed Road Expansion',
        type: 'road_work',
        status: 'active',
        geometry: {
          type: 'Polygon',
          coordinates: [[[55.2680, 25.2020], [55.2720, 25.2020], [55.2720, 25.2080], [55.2680, 25.2080], [55.2680, 25.2020]]]
        },
        startDate: new Date('2024-01-01'),
        endDate: new Date('2024-06-30'),
        impact: {
          laneClosures: 2,
          speedReduction: 20,
          detourRequired: false
        },
        contractor: 'Dubai Construction Co.',
        permit: 'DM-2024-001'
      }
    ]);

    updateMapLayers();
  };

  const updateMapLayers = useCallback(() => {
    if (!map.current || !mapLoaded) return;

    // Update traffic layer
    const trafficFeatures = trafficData.map(segment => ({
      type: 'Feature' as const,
      properties: {
        congestionLevel: segment.congestionLevel,
        speed: segment.speed,
        averageSpeed: segment.averageSpeed
      },
      geometry: {
        type: 'LineString' as const,
        coordinates: segment.coordinates
      }
    }));

    const trafficSource = map.current.getSource('traffic-data') as mapboxgl.GeoJSONSource;
    if (trafficSource) {
      trafficSource.setData({
        type: 'FeatureCollection',
        features: trafficFeatures
      });
    }

    // Add traffic layer if not exists
    if (!map.current.getLayer('traffic-layer')) {
      map.current.addLayer({
        id: 'traffic-layer',
        type: 'line',
        source: 'traffic-data',
        paint: {
          'line-color': [
            'case',
            ['==', ['get', 'congestionLevel'], 'free'], '#4CAF50',
            ['==', ['get', 'congestionLevel'], 'moderate'], '#FF9800',
            ['==', ['get', 'congestionLevel'], 'heavy'], '#FF5722',
            '#F44336' // severe
          ],
          'line-width': [
            'interpolate',
            ['linear'],
            ['zoom'],
            10, 2,
            15, 8
          ],
          'line-opacity': 0.8
        }
      });
    }

    // Update construction zones
    const constructionFeatures = constructionZones.map(zone => ({
      type: 'Feature' as const,
      properties: {
        name: zone.name,
        type: zone.type,
        status: zone.status,
        impact: zone.impact
      },
      geometry: zone.geometry
    }));

    const constructionSource = map.current.getSource('construction-zones') as mapboxgl.GeoJSONSource;
    if (constructionSource) {
      constructionSource.setData({
        type: 'FeatureCollection',
        features: constructionFeatures
      });
    }

    // Add construction zones layer if not exists
    if (!map.current.getLayer('construction-layer')) {
      map.current.addLayer({
        id: 'construction-layer',
        type: 'fill',
        source: 'construction-zones',
        paint: {
          'fill-color': '#FF9800',
          'fill-opacity': 0.3,
          'fill-outline-color': '#FF9800'
        }
      });
    }

    // Update sensors
    const sensorFeatures = sensorData.map(sensor => ({
      type: 'Feature' as const,
      properties: {
        id: sensor.id,
        type: sensor.type,
        status: sensor.status,
        value: sensor.data.value,
        unit: sensor.data.unit,
        quality: sensor.data.quality
      },
      geometry: {
        type: 'Point' as const,
        coordinates: sensor.position
      }
    }));

    const sensorsSource = map.current.getSource('sensors') as mapboxgl.GeoJSONSource;
    if (sensorsSource) {
      sensorsSource.setData({
        type: 'FeatureCollection',
        features: sensorFeatures
      });
    }

    // Add sensors layer if not exists
    if (!map.current.getLayer('sensors-layer')) {
      map.current.addLayer({
        id: 'sensors-layer',
        type: 'circle',
        source: 'sensors',
        paint: {
          'circle-radius': [
            'interpolate',
            ['linear'],
            ['zoom'],
            10, 4,
            15, 8
          ],
          'circle-color': [
            'case',
            ['==', ['get', 'status'], 'online'], '#4CAF50',
            ['==', ['get', 'status'], 'maintenance'], '#FF9800',
            '#F44336' // offline
          ],
          'circle-stroke-width': 2,
          'circle-stroke-color': '#fff',
          'circle-opacity': 0.8
        }
      });
    }

    // Add sensor coverage areas
    sensorData.forEach((sensor, index) => {
      if (sensor.coverage && !map.current!.getLayer(`sensor-coverage-${index}`)) {
        const coverageData = createCoverageArea(sensor);
        
        map.current!.addSource(`sensor-coverage-${index}`, {
          type: 'geojson',
          data: coverageData
        });

        map.current!.addLayer({
          id: `sensor-coverage-${index}`,
          type: 'fill',
          source: `sensor-coverage-${index}`,
          paint: {
            'fill-color': '#2196F3',
            'fill-opacity': 0.1,
            'fill-outline-color': '#2196F3'
          }
        });
      }
    });

  }, [mapLoaded, sensorData, trafficData, constructionZones]);

  const createCoverageArea = (sensor: SensorOverlay) => {
    const { position, coverage } = sensor;
    if (!coverage) return { type: 'FeatureCollection', features: [] };

    const [lng, lat] = position;
    const radius = coverage.radius;
    
    // Create circle for coverage area
    const points = 64;
    const coordinates = [];
    
    for (let i = 0; i < points; i++) {
      const angle = (i / points) * 2 * Math.PI;
      const dx = radius * Math.cos(angle) / 111320; // Convert meters to degrees
      const dy = radius * Math.sin(angle) / 110540;
      coordinates.push([lng + dx, lat + dy]);
    }
    coordinates.push(coordinates[0]); // Close the polygon

    return {
      type: 'FeatureCollection',
      features: [{
        type: 'Feature',
        properties: {},
        geometry: {
          type: 'Polygon',
          coordinates: [coordinates]
        }
      }]
    };
  };

  const toggleLayer = (layerId: string) => {
    setLayers(prev => prev.map(layer => 
      layer.id === layerId 
        ? { ...layer, visible: !layer.visible }
        : layer
    ));
    
    // Update map layer visibility
    if (map.current) {
      const mapLayerId = `${layerId}-layer`;
      const visibility = layers.find(l => l.id === layerId)?.visible ? 'none' : 'visible';
      
      if (map.current.getLayer(mapLayerId)) {
        map.current.setLayoutProperty(mapLayerId, 'visibility', visibility);
      }
    }
  };

  const updateLayerOpacity = (layerId: string, opacity: number) => {
    setLayers(prev => prev.map(layer => 
      layer.id === layerId 
        ? { ...layer, opacity }
        : layer
    ));
    
    // Update map layer opacity
    if (map.current) {
      const mapLayerId = `${layerId}-layer`;
      if (map.current.getLayer(mapLayerId)) {
        const layer = map.current.getLayer(mapLayerId);
        if (layer?.type === 'line') {
          map.current.setPaintProperty(mapLayerId, 'line-opacity', opacity);
        } else if (layer?.type === 'fill') {
          map.current.setPaintProperty(mapLayerId, 'fill-opacity', opacity);
        } else if (layer?.type === 'circle') {
          map.current.setPaintProperty(mapLayerId, 'circle-opacity', opacity);
        }
      }
    }
  };

  const toggle3D = () => {
    if (!map.current) return;
    
    const newPitch = is3DEnabled ? 0 : 60;
    const newBearing = is3DEnabled ? 0 : -17.6;
    
    map.current.easeTo({
      pitch: newPitch,
      bearing: newBearing,
      duration: 1000
    });
    
    setIs3DEnabled(!is3DEnabled);
  };

  const changeMapStyle = (style: 'streets' | 'satellite' | 'hybrid') => {
    if (!map.current) return;
    
    const styleUrls = {
      streets: 'mapbox://styles/mapbox/streets-v12',
      satellite: 'mapbox://styles/mapbox/satellite-v9',
      hybrid: 'mapbox://styles/mapbox/satellite-streets-v12'
    };
    
    map.current.setStyle(styleUrls[style]);
    setMapStyle(style);
    
    // Re-add custom layers after style change
    map.current.once('styledata', () => {
      initializeMapLayers();
      updateMapLayers();
    });
  };

  const refreshData = () => {
    loadMapData();
  };

  return (
    <MapContainer>
      <div ref={mapContainer} style={{ width: '100%', height: '100%' }} />
      
      {/* Map Controls */}
      <MapControls>
        <Tooltip title="Toggle 3D View">
          <IconButton
            onClick={toggle3D}
            sx={{
              backgroundColor: is3DEnabled ? 'primary.main' : 'background.paper',
              color: is3DEnabled ? 'primary.contrastText' : 'text.primary',
              '&:hover': { backgroundColor: is3DEnabled ? 'primary.dark' : 'action.hover' }
            }}
          >
            <ThreeDIcon />
          </IconButton>
        </Tooltip>
        
        <Tooltip title="Toggle Layer Controls">
          <IconButton
            onClick={() => setShowLayerControls(!showLayerControls)}
            sx={{ backgroundColor: 'background.paper' }}
          >
            <LayersIcon />
          </IconButton>
        </Tooltip>
        
        <Tooltip title="Refresh Data">
          <IconButton
            onClick={refreshData}
            sx={{ backgroundColor: 'background.paper' }}
          >
            <RefreshIcon />
          </IconButton>
        </Tooltip>
      </MapControls>

      {/* Layer Controls Panel */}
      {showLayerControls && (
        <LayerControls>
          <Typography variant="h6" gutterBottom>
            Map Layers
          </Typography>
          
          {/* Map Style Selection */}
          <Box sx={{ mb: 3 }}>
            <Typography variant="subtitle2" gutterBottom>
              Map Style
            </Typography>
            <Box sx={{ display: 'flex', gap: 1 }}>
              {(['streets', 'satellite', 'hybrid'] as const).map((style) => (
                <Chip
                  key={style}
                  label={style.charAt(0).toUpperCase() + style.slice(1)}
                  onClick={() => changeMapStyle(style)}
                  color={mapStyle === style ? 'primary' : 'default'}
                  variant={mapStyle === style ? 'filled' : 'outlined'}
                />
              ))}
            </Box>
          </Box>

          {/* Layer Controls */}
          {layers.map((layer) => (
            <Box key={layer.id} sx={{ mb: 2 }}>
              <FormControlLabel
                control={
                  <Switch
                    checked={layer.visible}
                    onChange={() => toggleLayer(layer.id)}
                    color="primary"
                  />
                }
                label={
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                    {layer.icon}
                    <Typography variant="body2">{layer.name}</Typography>
                  </Box>
                }
              />
              
              {layer.visible && (
                <Box sx={{ mt: 1, ml: 4 }}>
                  <Typography variant="caption" gutterBottom>
                    Opacity: {Math.round(layer.opacity * 100)}%
                  </Typography>
                  <Slider
                    value={layer.opacity}
                    onChange={(_, value) => updateLayerOpacity(layer.id, value as number)}
                    min={0}
                    max={1}
                    step={0.1}
                    size="small"
                    sx={{ color: layer.color }}
                  />
                </Box>
              )}
            </Box>
          ))}

          {/* Statistics */}
          <Box sx={{ mt: 3, pt: 2, borderTop: 1, borderColor: 'divider' }}>
            <Typography variant="subtitle2" gutterBottom>
              Layer Statistics
            </Typography>
            <Typography variant="caption" display="block">
              Sensors: {sensorData.length} active
            </Typography>
            <Typography variant="caption" display="block">
              Traffic Segments: {trafficData.length}
            </Typography>
            <Typography variant="caption" display="block">
              Construction Zones: {constructionZones.length}
            </Typography>
          </Box>
        </LayerControls>
      )}
    </MapContainer>
  );
};

export default AdvancedMapView;

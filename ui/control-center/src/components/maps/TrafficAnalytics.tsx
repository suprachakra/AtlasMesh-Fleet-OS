import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Grid,
  Chip,
  LinearProgress,
  Alert,
  IconButton,
  Tooltip,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
} from '@mui/material';
import {
  Traffic as TrafficIcon,
  Speed as SpeedIcon,
  Warning as WarningIcon,
  Timeline as TimelineIcon,
  Refresh as RefreshIcon,
  TrendingUp as TrendingUpIcon,
  TrendingDown as TrendingDownIcon,
} from '@mui/icons-material';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip as RechartsTooltip, ResponsiveContainer, BarChart, Bar } from 'recharts';

interface TrafficMetrics {
  averageSpeed: number;
  congestionIndex: number;
  incidentCount: number;
  flowRate: number; // vehicles per hour
  travelTimeIndex: number;
  co2Emissions: number;
  fuelConsumption: number;
}

interface TrafficTrend {
  timestamp: Date;
  speed: number;
  congestion: number;
  incidents: number;
  flow: number;
}

interface TrafficIncident {
  id: string;
  type: 'accident' | 'construction' | 'weather' | 'event' | 'breakdown';
  severity: 'low' | 'medium' | 'high' | 'critical';
  location: string;
  description: string;
  startTime: Date;
  estimatedDuration: number;
  affectedVehicles: number;
  delayMinutes: number;
  status: 'active' | 'clearing' | 'resolved';
}

interface CongestionHotspot {
  id: string;
  location: string;
  coordinates: [number, number];
  severity: number; // 0-100
  averageDelay: number;
  peakHours: string[];
  causes: string[];
  recommendedActions: string[];
}

const TrafficAnalytics: React.FC = () => {
  const [metrics, setMetrics] = useState<TrafficMetrics | null>(null);
  const [trends, setTrends] = useState<TrafficTrend[]>([]);
  const [incidents, setIncidents] = useState<TrafficIncident[]>([]);
  const [hotspots, setHotspots] = useState<CongestionHotspot[]>([]);
  const [loading, setLoading] = useState(true);
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());

  useEffect(() => {
    loadTrafficData();
    const interval = setInterval(loadTrafficData, 60000); // Update every minute
    return () => clearInterval(interval);
  }, []);

  const loadTrafficData = async () => {
    try {
      setLoading(true);
      
      // Load current metrics
      const metricsResponse = await fetch('/api/v1/traffic/metrics');
      const metricsData = await metricsResponse.json();
      setMetrics(metricsData);

      // Load trend data
      const trendsResponse = await fetch('/api/v1/traffic/trends?hours=24');
      const trendsData = await trendsResponse.json();
      setTrends(trendsData);

      // Load active incidents
      const incidentsResponse = await fetch('/api/v1/traffic/incidents?status=active');
      const incidentsData = await incidentsResponse.json();
      setIncidents(incidentsData);

      // Load congestion hotspots
      const hotspotsResponse = await fetch('/api/v1/traffic/hotspots');
      const hotspotsData = await hotspotsResponse.json();
      setHotspots(hotspotsData);

      setLastUpdate(new Date());
    } catch (error) {
      console.error('Failed to load traffic data:', error);
      // Load mock data
      loadMockData();
    } finally {
      setLoading(false);
    }
  };

  const loadMockData = () => {
    setMetrics({
      averageSpeed: 45.2,
      congestionIndex: 0.65,
      incidentCount: 8,
      flowRate: 2450,
      travelTimeIndex: 1.35,
      co2Emissions: 125.8,
      fuelConsumption: 89.2,
    });

    // Generate mock trend data for last 24 hours
    const now = new Date();
    const mockTrends: TrafficTrend[] = [];
    for (let i = 23; i >= 0; i--) {
      const timestamp = new Date(now.getTime() - i * 60 * 60 * 1000);
      mockTrends.push({
        timestamp,
        speed: 40 + Math.random() * 20 + Math.sin(i / 4) * 10,
        congestion: 0.3 + Math.random() * 0.4 + Math.sin(i / 6) * 0.2,
        incidents: Math.floor(Math.random() * 15),
        flow: 2000 + Math.random() * 1000 + Math.sin(i / 8) * 500,
      });
    }
    setTrends(mockTrends);

    setIncidents([
      {
        id: 'inc-001',
        type: 'accident',
        severity: 'high',
        location: 'Sheikh Zayed Road, Exit 41',
        description: 'Multi-vehicle collision blocking 2 lanes',
        startTime: new Date(Date.now() - 45 * 60 * 1000),
        estimatedDuration: 90,
        affectedVehicles: 150,
        delayMinutes: 25,
        status: 'active',
      },
      {
        id: 'inc-002',
        type: 'construction',
        severity: 'medium',
        location: 'Al Khaleej Road, near Dubai Mall',
        description: 'Lane closure for utility work',
        startTime: new Date(Date.now() - 2 * 60 * 60 * 1000),
        estimatedDuration: 240,
        affectedVehicles: 80,
        delayMinutes: 12,
        status: 'active',
      },
      {
        id: 'inc-003',
        type: 'weather',
        severity: 'low',
        location: 'Dubai-Al Ain Road',
        description: 'Reduced visibility due to sandstorm',
        startTime: new Date(Date.now() - 30 * 60 * 1000),
        estimatedDuration: 60,
        affectedVehicles: 45,
        delayMinutes: 8,
        status: 'clearing',
      },
    ]);

    setHotspots([
      {
        id: 'hot-001',
        location: 'Sheikh Zayed Road - Business Bay',
        coordinates: [55.2708, 25.2048],
        severity: 85,
        averageDelay: 18.5,
        peakHours: ['07:00-09:00', '17:00-19:30'],
        causes: ['High traffic volume', 'Limited lane capacity', 'Multiple exits'],
        recommendedActions: ['Dynamic lane management', 'Ramp metering', 'Alternative route promotion'],
      },
      {
        id: 'hot-002',
        location: 'Al Garhoud Bridge',
        coordinates: [55.3273, 25.2285],
        severity: 72,
        averageDelay: 12.3,
        peakHours: ['08:00-10:00', '16:30-18:30'],
        causes: ['Bridge bottleneck', 'Airport traffic'],
        recommendedActions: ['Traffic signal optimization', 'Public transport promotion'],
      },
    ]);
  };

  const getSeverityColor = (severity: string | number) => {
    if (typeof severity === 'string') {
      switch (severity) {
        case 'low': return 'success';
        case 'medium': return 'warning';
        case 'high': return 'error';
        case 'critical': return 'error';
        default: return 'default';
      }
    } else {
      if (severity >= 80) return 'error';
      if (severity >= 60) return 'warning';
      if (severity >= 40) return 'info';
      return 'success';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active': return 'error';
      case 'clearing': return 'warning';
      case 'resolved': return 'success';
      default: return 'default';
    }
  };

  const getCongestionLevel = (index: number) => {
    if (index >= 0.8) return { level: 'Severe', color: 'error' };
    if (index >= 0.6) return { level: 'Heavy', color: 'warning' };
    if (index >= 0.4) return { level: 'Moderate', color: 'info' };
    return { level: 'Light', color: 'success' };
  };

  const formatDuration = (minutes: number) => {
    const hours = Math.floor(minutes / 60);
    const mins = minutes % 60;
    return hours > 0 ? `${hours}h ${mins}m` : `${mins}m`;
  };

  if (loading && !metrics) {
    return (
      <Box sx={{ p: 3 }}>
        <Typography>Loading traffic analytics...</Typography>
        <LinearProgress sx={{ mt: 2 }} />
      </Box>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Typography variant="h4" component="h1">
          Traffic Analytics
        </Typography>
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
          <Typography variant="caption" color="text.secondary">
            Last updated: {lastUpdate.toLocaleTimeString()}
          </Typography>
          <Tooltip title="Refresh Data">
            <IconButton onClick={loadTrafficData} disabled={loading}>
              <RefreshIcon />
            </IconButton>
          </Tooltip>
        </Box>
      </Box>

      {/* Key Metrics */}
      <Grid container spacing={3} sx={{ mb: 4 }}>
        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                <SpeedIcon color="primary" sx={{ mr: 1 }} />
                <Typography variant="h6">Average Speed</Typography>
              </Box>
              <Typography variant="h4" color="primary">
                {metrics?.averageSpeed.toFixed(1)} km/h
              </Typography>
              <Box sx={{ display: 'flex', alignItems: 'center', mt: 1 }}>
                <TrendingUpIcon fontSize="small" color="success" />
                <Typography variant="caption" color="success.main">
                  +2.3% from yesterday
                </Typography>
              </Box>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                <TrafficIcon color="warning" sx={{ mr: 1 }} />
                <Typography variant="h6">Congestion</Typography>
              </Box>
              <Typography variant="h4" color="warning.main">
                {((metrics?.congestionIndex || 0) * 100).toFixed(0)}%
              </Typography>
              <Chip
                label={getCongestionLevel(metrics?.congestionIndex || 0).level}
                color={getCongestionLevel(metrics?.congestionIndex || 0).color as any}
                size="small"
                sx={{ mt: 1 }}
              />
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                <WarningIcon color="error" sx={{ mr: 1 }} />
                <Typography variant="h6">Active Incidents</Typography>
              </Box>
              <Typography variant="h4" color="error.main">
                {metrics?.incidentCount || 0}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                {incidents.filter(i => i.severity === 'high' || i.severity === 'critical').length} high priority
              </Typography>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} sm={6} md={3}>
          <Card>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                <TimelineIcon color="info" sx={{ mr: 1 }} />
                <Typography variant="h6">Flow Rate</Typography>
              </Box>
              <Typography variant="h4" color="info.main">
                {metrics?.flowRate.toLocaleString()}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                vehicles/hour
              </Typography>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Traffic Trends Chart */}
      <Card sx={{ mb: 4 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            24-Hour Traffic Trends
          </Typography>
          <ResponsiveContainer width="100%" height={300}>
            <LineChart data={trends}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis 
                dataKey="timestamp" 
                tickFormatter={(value) => new Date(value).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              />
              <YAxis yAxisId="speed" orientation="left" />
              <YAxis yAxisId="congestion" orientation="right" />
              <RechartsTooltip 
                labelFormatter={(value) => new Date(value).toLocaleString()}
                formatter={(value: any, name: string) => [
                  name === 'speed' ? `${value.toFixed(1)} km/h` : 
                  name === 'congestion' ? `${(value * 100).toFixed(0)}%` :
                  name === 'flow' ? `${value.toFixed(0)} veh/h` : value,
                  name === 'speed' ? 'Average Speed' :
                  name === 'congestion' ? 'Congestion Index' :
                  name === 'flow' ? 'Flow Rate' : name
                ]}
              />
              <Line 
                yAxisId="speed" 
                type="monotone" 
                dataKey="speed" 
                stroke="#2196F3" 
                strokeWidth={2}
                name="speed"
              />
              <Line 
                yAxisId="congestion" 
                type="monotone" 
                dataKey="congestion" 
                stroke="#FF9800" 
                strokeWidth={2}
                name="congestion"
              />
              <Line 
                yAxisId="speed" 
                type="monotone" 
                dataKey="flow" 
                stroke="#4CAF50" 
                strokeWidth={2}
                name="flow"
              />
            </LineChart>
          </ResponsiveContainer>
        </CardContent>
      </Card>

      <Grid container spacing={3}>
        {/* Active Incidents */}
        <Grid item xs={12} lg={8}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Active Traffic Incidents
              </Typography>
              <TableContainer>
                <Table>
                  <TableHead>
                    <TableRow>
                      <TableCell>Type</TableCell>
                      <TableCell>Location</TableCell>
                      <TableCell>Severity</TableCell>
                      <TableCell>Duration</TableCell>
                      <TableCell>Impact</TableCell>
                      <TableCell>Status</TableCell>
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {incidents.map((incident) => (
                      <TableRow key={incident.id}>
                        <TableCell>
                          <Chip 
                            label={incident.type.replace('_', ' ')} 
                            size="small" 
                            variant="outlined"
                          />
                        </TableCell>
                        <TableCell>{incident.location}</TableCell>
                        <TableCell>
                          <Chip 
                            label={incident.severity} 
                            color={getSeverityColor(incident.severity) as any}
                            size="small"
                          />
                        </TableCell>
                        <TableCell>
                          {formatDuration(Math.floor((Date.now() - incident.startTime.getTime()) / 60000))}
                        </TableCell>
                        <TableCell>
                          <Typography variant="body2">
                            {incident.affectedVehicles} vehicles
                          </Typography>
                          <Typography variant="caption" color="text.secondary">
                            +{incident.delayMinutes}min delay
                          </Typography>
                        </TableCell>
                        <TableCell>
                          <Chip 
                            label={incident.status} 
                            color={getStatusColor(incident.status) as any}
                            size="small"
                          />
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              </TableContainer>
            </CardContent>
          </Card>
        </Grid>

        {/* Congestion Hotspots */}
        <Grid item xs={12} lg={4}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Congestion Hotspots
              </Typography>
              {hotspots.map((hotspot) => (
                <Box key={hotspot.id} sx={{ mb: 3, p: 2, border: 1, borderColor: 'divider', borderRadius: 1 }}>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
                    <Typography variant="subtitle2" noWrap>
                      {hotspot.location}
                    </Typography>
                    <Chip 
                      label={`${hotspot.severity}%`}
                      color={getSeverityColor(hotspot.severity) as any}
                      size="small"
                    />
                  </Box>
                  
                  <Typography variant="body2" color="text.secondary" gutterBottom>
                    Avg. delay: {hotspot.averageDelay} minutes
                  </Typography>
                  
                  <Typography variant="caption" color="text.secondary" display="block" gutterBottom>
                    Peak hours: {hotspot.peakHours.join(', ')}
                  </Typography>
                  
                  <Box sx={{ mt: 1 }}>
                    <Typography variant="caption" color="text.secondary">
                      Main causes:
                    </Typography>
                    {hotspot.causes.slice(0, 2).map((cause, index) => (
                      <Chip 
                        key={index}
                        label={cause} 
                        size="small" 
                        variant="outlined"
                        sx={{ ml: 0.5, mt: 0.5, fontSize: '0.7rem' }}
                      />
                    ))}
                  </Box>
                </Box>
              ))}
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Environmental Impact */}
      <Card sx={{ mt: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Environmental Impact
          </Typography>
          <Grid container spacing={3}>
            <Grid item xs={12} sm={6}>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                <Typography variant="body1" sx={{ mr: 2 }}>
                  CO₂ Emissions:
                </Typography>
                <Typography variant="h6" color="error.main">
                  {metrics?.co2Emissions.toFixed(1)} tons/day
                </Typography>
              </Box>
              <LinearProgress 
                variant="determinate" 
                value={(metrics?.co2Emissions || 0) / 200 * 100} 
                color="error"
                sx={{ height: 8, borderRadius: 4 }}
              />
            </Grid>
            <Grid item xs={12} sm={6}>
              <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
                <Typography variant="body1" sx={{ mr: 2 }}>
                  Fuel Consumption:
                </Typography>
                <Typography variant="h6" color="warning.main">
                  {metrics?.fuelConsumption.toFixed(1)} L/100km
                </Typography>
              </Box>
              <LinearProgress 
                variant="determinate" 
                value={(metrics?.fuelConsumption || 0) / 120 * 100} 
                color="warning"
                sx={{ height: 8, borderRadius: 4 }}
              />
            </Grid>
          </Grid>
        </CardContent>
      </Card>
    </Box>
  );
};

export default TrafficAnalytics;

# Passenger App Adapter

## Purpose
The Passenger App Adapter provides integration with passenger mobile applications for autonomous vehicle fleet management in ride-hail operations. This adapter enables ride booking, real-time tracking, and passenger communication for robotaxi services.

## Features
- **Ride Booking**: Integration with passenger mobile apps for ride requests
- **Real-time Tracking**: Live vehicle location and ETA updates
- **Passenger Communication**: Two-way communication between passengers and vehicles
- **Ride Management**: Ride status updates and notifications
- **Rating System**: Integration with passenger rating and feedback systems
- **Accessibility**: Support for accessibility features and requirements

## Architecture
```
┌─────────────────────────────────────────────────────────────┐
│ Passenger App Adapter                                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐          │
│ │ Ride         │ │ Real-time    │ │ Passenger   │          │
│ │ Booking      │ │ Tracking     │ │ Communication│         │
│ └──────────────┘ └──────────────┘ └─────────────┘          │
│                                                             │
│ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐          │
│ │ Ride         │ │ Rating       │ │ Accessibility│         │
│ │ Management   │ │ System       │ │ Features    │          │
│ └──────────────┘ └──────────────┘ └─────────────┘          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Configuration
The adapter supports configuration through YAML files and environment variables:

```yaml
passenger_app:
  enabled: true
  api_endpoint: "https://passenger-app.example.com/api"
  websocket_url: "wss://passenger-app.example.com/ws"
  push_notifications:
    enabled: true
    provider: "firebase"
    api_key: "${FIREBASE_API_KEY}"
  ride_booking:
    max_advance_booking_hours: 24
    cancellation_window_minutes: 5
  tracking:
    update_interval_seconds: 5
    accuracy_threshold_meters: 10
  accessibility:
    enabled: true
    features: ["voice_announcements", "large_text", "high_contrast"]
```

## API Endpoints
- `POST /api/v1/passenger/ride/request` - Request a ride
- `GET /api/v1/passenger/ride/status` - Get ride status
- `POST /api/v1/passenger/ride/cancel` - Cancel a ride
- `GET /api/v1/passenger/ride/track` - Track ride location
- `POST /api/v1/passenger/ride/rate` - Rate a ride

## Mobile App Integration
The adapter provides SDKs for iOS and Android:
- Real-time ride tracking
- Push notifications
- In-app communication
- Payment integration
- Accessibility features

## Deployment
The adapter can be deployed using Docker or Kubernetes:

```bash
# Docker
docker build -t passenger-app-adapter .
docker run -p 8080:8080 passenger-app-adapter

# Kubernetes
kubectl apply -f k8s/deployment.yaml
```

## Monitoring
The adapter provides Prometheus metrics for monitoring:
- `passenger_requests_total` - Total passenger requests
- `ride_bookings_total` - Total ride bookings
- `tracking_updates_total` - Total tracking updates
- `passenger_satisfaction_score` - Passenger satisfaction score

## Testing
Run the test suite:

```bash
go test ./...
```

## License
This adapter is part of the AtlasMesh Fleet OS project and is subject to the project's license terms.

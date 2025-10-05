# Control Center UI

> **TL;DR:** Real-time fleet monitoring and control interface for autonomous vehicle operations

## 📊 **Architecture Overview**

### 🖥️ **Where it fits** - User Interface Layer
```mermaid
graph TB
    subgraph "Users"
        FleetOp[👤 Fleet Operator]
        SafetyOp[🚨 Safety Operator]
        Supervisor[👑 Fleet Supervisor]
    end
    
    subgraph "Control Center UI"
        WebApp[⚛️ React Web App]
        StateManager[🗂️ Redux Store]
        APIClient[🔌 API Client]
        WSClient[📡 WebSocket Client]
    end
    
    subgraph "UI Components"
        Dashboard[📊 Fleet Dashboard]
        VehicleGrid[🚗 Vehicle Grid]
        LiveMap[🗺️ Live Map]
        EmergencyBtn[🚨 Emergency Controls]
        Alerts[🔔 Alert System]
    end
    
    subgraph "Backend Services"
        APIGateway[🚪 API Gateway]
        FleetManager[🚛 Fleet Manager]
        VehicleGW[🌐 Vehicle Gateway]
        Auth[🔐 Auth Service]
    end
    
    subgraph "Real-time Data"
        WebSocketAPI[📡 WebSocket API]
        TelemetryStream[📊 Telemetry Stream]
        EventBus[📨 Event Bus]
    end
    
    FleetOp --> WebApp
    SafetyOp --> WebApp
    Supervisor --> WebApp
    
    WebApp --> StateManager
    WebApp --> APIClient
    WebApp --> WSClient
    
    StateManager --> Dashboard
    StateManager --> VehicleGrid
    StateManager --> LiveMap
    StateManager --> EmergencyBtn
    StateManager --> Alerts
    
    APIClient --> APIGateway
    WSClient --> WebSocketAPI
    
    APIGateway --> FleetManager
    APIGateway --> Auth
    WebSocketAPI --> VehicleGW
    VehicleGW --> TelemetryStream
    TelemetryStream --> EventBus
    
    classDef ui fill:#e3f2fd,stroke:#1976d2,stroke-width:3px
    classDef user fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    classDef component fill:#e8f5e8,stroke:#388e3c,stroke-width:2px
    classDef backend fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef realtime fill:#ffebee,stroke:#c62828,stroke-width:2px
    
    class WebApp,StateManager,APIClient,WSClient ui
    class FleetOp,SafetyOp,Supervisor user
    class Dashboard,VehicleGrid,LiveMap,EmergencyBtn,Alerts component
    class APIGateway,FleetManager,VehicleGW,Auth backend
    class WebSocketAPI,TelemetryStream,EventBus realtime

    style Users fill:transparent
    style subGraph1 fill:transparent
    style subGraph2 fill:transparent
    style subGraph3 fill:transparent
    style subGraph4 fill:transparent
```

### ⚡ **Critical User Flow** - Emergency Stop
```mermaid
journey
    title Emergency Stop - Safety Operator Response
    section Threat Detection
      Monitor fleet status: 5: Safety Operator
      Identify safety hazard: 3: Safety Operator
      Assess threat level: 2: Safety Operator
    section Emergency Response
      Click emergency stop button: 5: Safety Operator
      Confirm emergency action: 4: Safety Operator
      Monitor command execution: 4: Safety Operator
    section Verification & Follow-up
      Verify vehicle stopped: 5: Safety Operator
      Check vehicle safety status: 5: Safety Operator
      Document incident details: 3: Safety Operator
      Coordinate recovery actions: 4: Safety Operator
```

### 🔌 **API Integration** - Service Communication
```mermaid
flowchart TB
    subgraph "Control Center UI"
        LoginForm[🔐 Login Form]
        Dashboard[📊 Dashboard]
        VehicleCard[🚗 Vehicle Card]
        EmergencyBtn[🚨 Emergency Button]
        MapView[🗺️ Map View]
    end
    
    subgraph "API Gateway"
        AuthEndpoint[🔐 /api/v1/auth/*]
        FleetEndpoint[🚛 /api/v1/fleets/*]
        VehicleEndpoint[🚗 /api/v1/vehicles/*]
        EmergencyEndpoint[🚨 /api/v1/emergency/*]
        TelemetryWS[📡 /ws/telemetry]
    end
    
    subgraph "Backend Services"
        AuthService[🔐 Auth Service]
        FleetManager[🚛 Fleet Manager]
        VehicleGateway[🌐 Vehicle Gateway]
        TelemetryIngest[📊 Telemetry Ingest]
    end
    
    subgraph "Data Sources"
        UserDB[(👤 User Database)]
        FleetDB[(🚛 Fleet Database)]
        VehicleState[(🚗 Vehicle State)]
        TelemetryStream[(📊 Telemetry Stream)]
    end
    
    %% Authentication Flow
    LoginForm -->|POST /login| AuthEndpoint
    AuthEndpoint --> AuthService
    AuthService --> UserDB
    
    %% Fleet Data Flow
    Dashboard -->|GET /fleets| FleetEndpoint
    FleetEndpoint --> FleetManager
    FleetManager --> FleetDB
    
    %% Vehicle Control Flow
    VehicleCard -->|POST /vehicles/id/commands| VehicleEndpoint
    VehicleEndpoint --> FleetManager
    FleetManager --> VehicleGateway
    VehicleGateway --> VehicleState
    
    %% Emergency Flow
    EmergencyBtn -->|POST /emergency/stop| EmergencyEndpoint
    EmergencyEndpoint --> VehicleGateway
    VehicleGateway --> VehicleState
    
    %% Real-time Data Flow
    MapView -->|WebSocket| TelemetryWS
    TelemetryWS --> TelemetryIngest
    TelemetryIngest --> TelemetryStream
    
    classDef ui fill:#e3f2fd,stroke:#1976d2,stroke-width:2px
    classDef api fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef service fill:#e8f5e8,stroke:#388e3c,stroke-width:2px
    classDef data fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class LoginForm,Dashboard,VehicleCard,EmergencyBtn,MapView ui
    class AuthEndpoint,FleetEndpoint,VehicleEndpoint,EmergencyEndpoint,TelemetryWS api
    class AuthService,FleetManager,VehicleGateway,TelemetryIngest service
    class UserDB,FleetDB,VehicleState,TelemetryStream data

    style subGraph0 fill:transparent
    style subGraph1 fill:transparent
    style subGraph2 fill:transparent
    style subGraph3 fill:transparent
```

## 🔗 **Integration Contracts**

| Component | API Endpoint | Method | Purpose |
|-----------|--------------|--------|---------|
| `LoginForm` | `/api/v1/auth/login` | `POST` | User authentication |
| `Dashboard` | `/api/v1/fleets` | `GET` | Fleet overview data |
| `VehicleGrid` | `/api/v1/vehicles` | `GET` | Vehicle status list |
| `EmergencyControls` | `/api/v1/emergency/stop` | `POST` | Emergency stop command |
| `LiveMap` | `/ws/telemetry` | `WebSocket` | Real-time vehicle positions |

## 🚀 **Quick Start**

### **Development**
```bash
# Install dependencies
npm install

# Start development server
npm run dev

# Open in browser
open http://localhost:3000

# Run Storybook
npm run storybook
```

### **Build & Deploy**
```bash
# Build for production
npm run build

# Preview production build
npm run preview

# Deploy to staging
npm run deploy:staging
```

## 📈 **UX Metrics & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **First Paint** | <1.5s | 1.2s ✅ |
| **Time to Interactive** | <3s | 2.4s ✅ |
| **Bundle Size** | <500KB | 420KB ✅ |
| **Accessibility Score** | 95+ | 98 ✅ |

**🎯 Core Web Vitals:** LCP 1.2s | FID 45ms | CLS 0.05

## 🎨 **Component Architecture**

### **State Management**
- **Global State:** Redux Toolkit for fleet and vehicle data
- **Server State:** React Query for API data caching and synchronization
- **Real-time State:** WebSocket integration for live telemetry
- **Local State:** React useState for component-specific state

### **Design System**
```typescript
// Theme Configuration
export const theme = {
  colors: {
    primary: '#1976d2',      // AtlasMesh Blue
    secondary: '#388e3c',    // Success Green
    accent: '#f57c00',       // Warning Orange
    danger: '#d32f2f',       // Emergency Red
  },
  typography: {
    fontFamily: 'Inter, system-ui, sans-serif',
  },
  breakpoints: {
    sm: '640px', md: '768px', lg: '1024px', xl: '1280px'
  }
}
```

## ♿ **Accessibility & Internationalization**

### **WCAG 2.2 AA Compliance**
- **Keyboard Navigation:** Full keyboard accessibility with focus management
- **Screen Reader:** ARIA labels, roles, and live regions for dynamic content
- **Color Contrast:** 4.5:1 minimum contrast ratio for all text
- **Focus Management:** Logical tab order and visible focus indicators

### **Internationalization**
```typescript
// Supported locales
const locales = ['en-US', 'ar-AE'] // English + Arabic (UAE)

// Emergency text in both languages
const emergencyText = {
  'en-US': 'EMERGENCY STOP',
  'ar-AE': 'توقف طارئ'
}
```

## 🧪 **Testing Strategy**

### **Test Coverage**
- **Unit Tests:** 85% (Vitest + React Testing Library)
- **Component Tests:** 78% (Storybook + Chromatic)
- **E2E Tests:** 72% (Playwright)
- **Accessibility Tests:** axe-core automated testing

### **Testing Commands**
```bash
# All tests
npm run test:all

# Unit tests
npm run test:unit

# E2E tests
npm run test:e2e

# Accessibility tests
npm run test:a11y
```

## 📊 **Monitoring & Analytics**

- **Performance Monitoring:** Web Vitals + Sentry
- **User Analytics:** Google Analytics 4 tracking
- **Error Tracking:** Sentry for JavaScript errors
- **Real-time Metrics:** DataDog RUM

## 🆘 **Troubleshooting**

| Issue | Solution |
|-------|----------|
| Slow map rendering | Check network, verify API key limits |
| WebSocket disconnections | Verify backend connectivity, check auth tokens |
| Emergency button unresponsive | Check user permissions, verify API connectivity |
| Arabic text rendering | Verify font loading, check CSS direction properties |

---

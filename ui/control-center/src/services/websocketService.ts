/**
 * WebSocket Service for Real-time Communication
 * Handles connection to AtlasMesh Fleet OS backend services
 */

export interface WebSocketMessage {
  type: string;
  data: any;
  timestamp: number;
  correlationId?: string;
}

export interface WebSocketConfig {
  url: string;
  protocols?: string[];
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
  heartbeatInterval?: number;
}

export interface VehicleTelemetry {
  vehicleId: string;
  timestamp: number;
  location: {
    latitude: number;
    longitude: number;
    altitude?: number;
    heading?: number;
    speed?: number;
  };
  status: {
    batteryLevel: number;
    fuelLevel: number;
    healthScore: number;
    operationalStatus: string;
  };
  sensors: {
    temperature: number;
    humidity?: number;
    pressure?: number;
  };
  alerts: Array<{
    type: string;
    severity: 'low' | 'medium' | 'high' | 'critical';
    message: string;
    timestamp: number;
  }>;
}

export interface FleetEvent {
  eventId: string;
  type: 'vehicle_status_change' | 'trip_started' | 'trip_completed' | 'maintenance_required' | 'emergency_alert';
  vehicleId: string;
  timestamp: number;
  data: any;
  severity: 'info' | 'warning' | 'error' | 'critical';
}

class WebSocketService {
  private static instance: WebSocketService;
  private ws: WebSocket | null = null;
  private config: WebSocketConfig;
  private isConnected: boolean = false;
  private reconnectAttempts: number = 0;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private heartbeatTimer: NodeJS.Timeout | null = null;
  private messageCallbacks: Map<string, ((message: WebSocketMessage) => void)[]> = new Map();
  private connectionCallbacks: ((connected: boolean) => void)[] = [];

  private constructor(config: WebSocketConfig) {
    this.config = {
      reconnectInterval: 5000,
      maxReconnectAttempts: 10,
      heartbeatInterval: 30000,
      ...config,
    };
  }

  public static getInstance(config?: WebSocketConfig): WebSocketService {
    if (!WebSocketService.instance) {
      if (!config) {
        throw new Error('WebSocket configuration is required for first initialization');
      }
      WebSocketService.instance = new WebSocketService(config);
    }
    return WebSocketService.instance;
  }

  /**
   * Connect to WebSocket server
   */
  public async connect(): Promise<boolean> {
    try {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        console.log('WebSocket already connected');
        return true;
      }

      console.log('Connecting to WebSocket:', this.config.url);
      this.ws = new WebSocket(this.config.url, this.config.protocols);

      this.ws.onopen = this.handleOpen.bind(this);
      this.ws.onmessage = this.handleMessage.bind(this);
      this.ws.onclose = this.handleClose.bind(this);
      this.ws.onerror = this.handleError.bind(this);

      return new Promise((resolve) => {
        const timeout = setTimeout(() => {
          resolve(false);
        }, 10000); // 10 second timeout

        this.ws!.addEventListener('open', () => {
          clearTimeout(timeout);
          resolve(true);
        });
      });
    } catch (error) {
      console.error('WebSocket connection error:', error);
      return false;
    }
  }

  /**
   * Disconnect from WebSocket server
   */
  public disconnect(): void {
    this.clearTimers();
    
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
    
    this.isConnected = false;
    this.notifyConnectionCallbacks(false);
  }

  /**
   * Send message to WebSocket server
   */
  public send(message: WebSocketMessage): boolean {
    if (!this.isConnected || !this.ws) {
      console.warn('WebSocket not connected, cannot send message');
      return false;
    }

    try {
      const messageStr = JSON.stringify({
        ...message,
        timestamp: Date.now(),
      });
      
      this.ws.send(messageStr);
      console.log('WebSocket message sent:', message.type);
      return true;
    } catch (error) {
      console.error('Error sending WebSocket message:', error);
      return false;
    }
  }

  /**
   * Subscribe to specific message types
   */
  public subscribe(messageType: string, callback: (message: WebSocketMessage) => void): void {
    if (!this.messageCallbacks.has(messageType)) {
      this.messageCallbacks.set(messageType, []);
    }
    this.messageCallbacks.get(messageType)!.push(callback);
  }

  /**
   * Unsubscribe from message type
   */
  public unsubscribe(messageType: string, callback: (message: WebSocketMessage) => void): void {
    const callbacks = this.messageCallbacks.get(messageType);
    if (callbacks) {
      const index = callbacks.indexOf(callback);
      if (index > -1) {
        callbacks.splice(index, 1);
      }
    }
  }

  /**
   * Subscribe to connection status changes
   */
  public onConnectionChange(callback: (connected: boolean) => void): void {
    this.connectionCallbacks.push(callback);
  }

  /**
   * Get connection status
   */
  public getConnectionStatus(): boolean {
    return this.isConnected;
  }

  /**
   * Send heartbeat to keep connection alive
   */
  private sendHeartbeat(): void {
    if (this.isConnected) {
      this.send({
        type: 'heartbeat',
        data: { timestamp: Date.now() },
      });
    }
  }

  /**
   * Handle WebSocket open event
   */
  private handleOpen(): void {
    console.log('WebSocket connected');
    this.isConnected = true;
    this.reconnectAttempts = 0;
    this.clearTimers();
    this.startHeartbeat();
    this.notifyConnectionCallbacks(true);
  }

  /**
   * Handle WebSocket message event
   */
  private handleMessage(event: MessageEvent): void {
    try {
      const message: WebSocketMessage = JSON.parse(event.data);
      
      // Handle heartbeat response
      if (message.type === 'heartbeat_response') {
        console.log('Heartbeat received');
        return;
      }

      // Notify subscribers
      const callbacks = this.messageCallbacks.get(message.type);
      if (callbacks) {
        callbacks.forEach(callback => {
          try {
            callback(message);
          } catch (error) {
            console.error('Error in WebSocket message callback:', error);
          }
        });
      }

      // Notify wildcard subscribers
      const wildcardCallbacks = this.messageCallbacks.get('*');
      if (wildcardCallbacks) {
        wildcardCallbacks.forEach(callback => {
          try {
            callback(message);
          } catch (error) {
            console.error('Error in WebSocket wildcard callback:', error);
          }
        });
      }

    } catch (error) {
      console.error('Error parsing WebSocket message:', error);
    }
  }

  /**
   * Handle WebSocket close event
   */
  private handleClose(event: CloseEvent): void {
    console.log('WebSocket closed:', event.code, event.reason);
    this.isConnected = false;
    this.clearTimers();
    this.notifyConnectionCallbacks(false);
    
    // Attempt to reconnect if not manually closed
    if (event.code !== 1000 && this.reconnectAttempts < this.config.maxReconnectAttempts!) {
      this.scheduleReconnect();
    }
  }

  /**
   * Handle WebSocket error event
   */
  private handleError(error: Event): void {
    console.error('WebSocket error:', error);
    this.isConnected = false;
    this.notifyConnectionCallbacks(false);
  }

  /**
   * Schedule reconnection attempt
   */
  private scheduleReconnect(): void {
    if (this.reconnectTimer) {
      return; // Already scheduled
    }

    this.reconnectAttempts++;
    const delay = this.config.reconnectInterval! * Math.pow(2, this.reconnectAttempts - 1);
    
    console.log(`Scheduling reconnection attempt ${this.reconnectAttempts} in ${delay}ms`);
    
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, delay);
  }

  /**
   * Start heartbeat timer
   */
  private startHeartbeat(): void {
    this.heartbeatTimer = setInterval(() => {
      this.sendHeartbeat();
    }, this.config.heartbeatInterval);
  }

  /**
   * Clear all timers
   */
  private clearTimers(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }

  /**
   * Notify connection status callbacks
   */
  private notifyConnectionCallbacks(connected: boolean): void {
    this.connectionCallbacks.forEach(callback => {
      try {
        callback(connected);
      } catch (error) {
        console.error('Error in connection callback:', error);
      }
    });
  }

  /**
   * Send vehicle command
   */
  public sendVehicleCommand(vehicleId: string, command: string, data?: any): boolean {
    return this.send({
      type: 'vehicle_command',
      data: {
        vehicleId,
        command,
        data,
      },
    });
  }

  /**
   * Request fleet status
   */
  public requestFleetStatus(): boolean {
    return this.send({
      type: 'fleet_status_request',
      data: {},
    });
  }

  /**
   * Subscribe to vehicle telemetry
   */
  public subscribeToVehicleTelemetry(vehicleId: string, callback: (telemetry: VehicleTelemetry) => void): void {
    this.subscribe(`vehicle_telemetry_${vehicleId}`, callback);
    this.subscribe('vehicle_telemetry', callback);
  }

  /**
   * Subscribe to fleet events
   */
  public subscribeToFleetEvents(callback: (event: FleetEvent) => void): void {
    this.subscribe('fleet_event', callback);
  }

  /**
   * Subscribe to emergency alerts
   */
  public subscribeToEmergencyAlerts(callback: (alert: any) => void): void {
    this.subscribe('emergency_alert', callback);
  }

  /**
   * Send emergency stop command
   */
  public sendEmergencyStop(vehicleId: string): boolean {
    return this.sendVehicleCommand(vehicleId, 'emergency_stop', {
      timestamp: Date.now(),
      reason: 'manual_override',
    });
  }

  /**
   * Send maintenance request
   */
  public sendMaintenanceRequest(vehicleId: string, maintenanceType: string, description: string): boolean {
    return this.send({
      type: 'maintenance_request',
      data: {
        vehicleId,
        maintenanceType,
        description,
        timestamp: Date.now(),
      },
    });
  }
}

export default WebSocketService;

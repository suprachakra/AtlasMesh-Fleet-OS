import { useEffect, useState, useCallback, useRef } from 'react';
import WebSocketService, { WebSocketMessage, VehicleTelemetry, FleetEvent } from '../services/websocketService';

export interface UseWebSocketOptions {
  autoConnect?: boolean;
  reconnectOnMount?: boolean;
}

export interface UseWebSocketReturn {
  isConnected: boolean;
  connect: () => Promise<boolean>;
  disconnect: () => void;
  send: (message: WebSocketMessage) => boolean;
  subscribe: (messageType: string, callback: (message: WebSocketMessage) => void) => void;
  unsubscribe: (messageType: string, callback: (message: WebSocketMessage) => void) => void;
  sendVehicleCommand: (vehicleId: string, command: string, data?: any) => boolean;
  requestFleetStatus: () => boolean;
  subscribeToVehicleTelemetry: (vehicleId: string, callback: (telemetry: VehicleTelemetry) => void) => void;
  subscribeToFleetEvents: (callback: (event: FleetEvent) => void) => void;
  subscribeToEmergencyAlerts: (callback: (alert: any) => void) => void;
  sendEmergencyStop: (vehicleId: string) => boolean;
  sendMaintenanceRequest: (vehicleId: string, maintenanceType: string, description: string) => boolean;
}

/**
 * React hook for WebSocket communication
 * Provides real-time communication with AtlasMesh Fleet OS backend
 */
export const useWebSocket = (options: UseWebSocketOptions = {}): UseWebSocketReturn => {
  const {
    autoConnect = true,
    reconnectOnMount = true,
  } = options;

  const [isConnected, setIsConnected] = useState(false);
  const wsServiceRef = useRef<WebSocketService | null>(null);
  const callbacksRef = useRef<Map<string, ((message: WebSocketMessage) => void)[]>>(new Map());

  // Initialize WebSocket service
  useEffect(() => {
    if (!wsServiceRef.current) {
      const config = {
        url: process.env.REACT_APP_WEBSOCKET_URL || 'ws://localhost:8080/ws',
        reconnectInterval: 5000,
        maxReconnectAttempts: 10,
        heartbeatInterval: 30000,
      };

      wsServiceRef.current = WebSocketService.getInstance(config);
      
      // Subscribe to connection status changes
      wsServiceRef.current.onConnectionChange((connected) => {
        setIsConnected(connected);
      });

      // Auto-connect if enabled
      if (autoConnect) {
        wsServiceRef.current.connect();
      }
    }

    return () => {
      if (wsServiceRef.current && !reconnectOnMount) {
        wsServiceRef.current.disconnect();
      }
    };
  }, [autoConnect, reconnectOnMount]);

  // Connect to WebSocket
  const connect = useCallback(async (): Promise<boolean> => {
    if (!wsServiceRef.current) {
      return false;
    }
    return wsServiceRef.current.connect();
  }, []);

  // Disconnect from WebSocket
  const disconnect = useCallback((): void => {
    if (wsServiceRef.current) {
      wsServiceRef.current.disconnect();
    }
  }, []);

  // Send message
  const send = useCallback((message: WebSocketMessage): boolean => {
    if (!wsServiceRef.current) {
      return false;
    }
    return wsServiceRef.current.send(message);
  }, []);

  // Subscribe to message type
  const subscribe = useCallback((messageType: string, callback: (message: WebSocketMessage) => void): void => {
    if (!wsServiceRef.current) {
      return;
    }

    // Store callback reference for cleanup
    if (!callbacksRef.current.has(messageType)) {
      callbacksRef.current.set(messageType, []);
    }
    callbacksRef.current.get(messageType)!.push(callback);

    // Subscribe to WebSocket service
    wsServiceRef.current.subscribe(messageType, callback);
  }, []);

  // Unsubscribe from message type
  const unsubscribe = useCallback((messageType: string, callback: (message: WebSocketMessage) => void): void => {
    if (!wsServiceRef.current) {
      return;
    }

    // Remove callback reference
    const callbacks = callbacksRef.current.get(messageType);
    if (callbacks) {
      const index = callbacks.indexOf(callback);
      if (index > -1) {
        callbacks.splice(index, 1);
      }
    }

    // Unsubscribe from WebSocket service
    wsServiceRef.current.unsubscribe(messageType, callback);
  }, []);

  // Send vehicle command
  const sendVehicleCommand = useCallback((vehicleId: string, command: string, data?: any): boolean => {
    if (!wsServiceRef.current) {
      return false;
    }
    return wsServiceRef.current.sendVehicleCommand(vehicleId, command, data);
  }, []);

  // Request fleet status
  const requestFleetStatus = useCallback((): boolean => {
    if (!wsServiceRef.current) {
      return false;
    }
    return wsServiceRef.current.requestFleetStatus();
  }, []);

  // Subscribe to vehicle telemetry
  const subscribeToVehicleTelemetry = useCallback((vehicleId: string, callback: (telemetry: VehicleTelemetry) => void): void => {
    if (!wsServiceRef.current) {
      return;
    }
    wsServiceRef.current.subscribeToVehicleTelemetry(vehicleId, callback);
  }, []);

  // Subscribe to fleet events
  const subscribeToFleetEvents = useCallback((callback: (event: FleetEvent) => void): void => {
    if (!wsServiceRef.current) {
      return;
    }
    wsServiceRef.current.subscribeToFleetEvents(callback);
  }, []);

  // Subscribe to emergency alerts
  const subscribeToEmergencyAlerts = useCallback((callback: (alert: any) => void): void => {
    if (!wsServiceRef.current) {
      return;
    }
    wsServiceRef.current.subscribeToEmergencyAlerts(callback);
  }, []);

  // Send emergency stop
  const sendEmergencyStop = useCallback((vehicleId: string): boolean => {
    if (!wsServiceRef.current) {
      return false;
    }
    return wsServiceRef.current.sendEmergencyStop(vehicleId);
  }, []);

  // Send maintenance request
  const sendMaintenanceRequest = useCallback((vehicleId: string, maintenanceType: string, description: string): boolean => {
    if (!wsServiceRef.current) {
      return false;
    }
    return wsServiceRef.current.sendMaintenanceRequest(vehicleId, maintenanceType, description);
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (wsServiceRef.current) {
        // Unsubscribe all callbacks
        callbacksRef.current.forEach((callbacks, messageType) => {
          callbacks.forEach(callback => {
            wsServiceRef.current?.unsubscribe(messageType, callback);
          });
        });
        callbacksRef.current.clear();
      }
    };
  }, []);

  return {
    isConnected,
    connect,
    disconnect,
    send,
    subscribe,
    unsubscribe,
    sendVehicleCommand,
    requestFleetStatus,
    subscribeToVehicleTelemetry,
    subscribeToFleetEvents,
    subscribeToEmergencyAlerts,
    sendEmergencyStop,
    sendMaintenanceRequest,
  };
};

export default useWebSocket;

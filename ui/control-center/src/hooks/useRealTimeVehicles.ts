import { useState, useEffect, useRef, useCallback, useMemo } from 'react'

// Real-time vehicle data integration for AtlasMesh Fleet OS
// Abu Dhabi autonomous vehicle fleet operations

export interface VehicleLocation {
  lat: number
  lng: number
  heading?: number
  speed?: number
  timestamp: number
}

export interface RealTimeVehicle {
  id: string
  name: string
  type: 'sedan' | 'suv' | 'truck' | 'bus'
  status: 'operational' | 'warning' | 'critical' | 'offline'
  location: VehicleLocation
  batteryLevel?: number
  lastUpdate: number
  connectionStatus: 'connected' | 'disconnected'
}

export interface VehicleCommand {
  type: 'start' | 'stop' | 'emergency_stop'
  payload?: Record<string, unknown>
}

const DEFAULT_WS_URL = 'ws://localhost:8080/ws/vehicles'

export const useRealTimeVehicles = () => {
  const [vehicles, setVehicles] = useState<RealTimeVehicle[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [connectionStatus, setConnectionStatus] = useState<'connected' | 'disconnected'>('disconnected')

  const wsRef = useRef<WebSocket | null>(null)

  const connect = useCallback(() => {
    try {
      const ws = new WebSocket(DEFAULT_WS_URL)
      wsRef.current = ws

      ws.onopen = () => {
        setConnectionStatus('connected')
        setLoading(false)
        setError(null)
      }

      ws.onmessage = (event) => {
        const data = JSON.parse(event.data)
        if (data.type === 'vehicle_update') {
          setVehicles(prev => {
            const updated = [...prev]
            const index = updated.findIndex(v => v.id === data.vehicle.id)
            if (index >= 0) {
              updated[index] = data.vehicle
            } else {
              updated.push(data.vehicle)
            }
            return updated
          })
        }
      }

      ws.onclose = () => {
        setConnectionStatus('disconnected')
      }

      ws.onerror = () => {
        setError('Connection error')
        setConnectionStatus('disconnected')
      }

    } catch (err) {
      setError('Failed to connect')
      setConnectionStatus('disconnected')
    }
  }, [])

  const sendCommand = useCallback(async (vehicleId: string, command: VehicleCommand) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({
        type: 'vehicle_command',
        vehicleId,
        command
      }))
    }
  }, [])

  useEffect(() => {
    connect()
    return () => {
      if (wsRef.current) {
        wsRef.current.close()
      }
    }
  }, [connect])

  const stats = useMemo(() => ({
    total: vehicles.length,
    operational: vehicles.filter(v => v.status === 'operational').length,
    warning: vehicles.filter(v => v.status === 'warning').length,
    critical: vehicles.filter(v => v.status === 'critical').length,
    offline: vehicles.filter(v => v.status === 'offline').length
  }), [vehicles])

  return {
    vehicles,
    loading,
    error,
    connectionStatus,
    stats,
    sendCommand
  }
}

export default useRealTimeVehicles
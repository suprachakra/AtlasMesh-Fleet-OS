// AtlasMesh Fleet OS - Telemetry Service
// Real-time telemetry data integration for Abu Dhabi autonomous vehicle fleet

import { useState, useEffect } from 'react'

export interface TelemetryData {
  vehicleId: string
  timestamp: number
  location: {
    lat: number
    lng: number
    heading: number
    speed: number
    altitude?: number
  }
  sensors: {
    lidar?: {
      status: 'operational' | 'warning' | 'critical' | 'offline'
      pointCloudSize?: number
      range?: number
    }
    camera?: {
      status: 'operational' | 'warning' | 'critical' | 'offline'
      resolution?: string
      fps?: number
    }
    radar?: {
      status: 'operational' | 'warning' | 'critical' | 'offline'
      range?: number
      targets?: number
    }
    gps?: {
      status: 'operational' | 'warning' | 'critical' | 'offline'
      satellites?: number
      accuracy?: number
    }
  }
  vehicle: {
    batteryLevel?: number
    fuelLevel?: number
    engineTemp?: number
    speed: number
    odometer?: number
    diagnostics?: Record<string, unknown>
  }
  environment: {
    weather?: {
      temperature?: number
      humidity?: number
      visibility?: number
      conditions?: string
    }
    traffic?: {
      density?: 'low' | 'medium' | 'high'
      incidents?: number
    }
  }
}

export interface TelemetrySubscription {
  vehicleId: string
  callback: (data: TelemetryData) => void
  filters?: {
    dataTypes?: string[]
    updateInterval?: number
  }
}

class TelemetryService {
  private ws: WebSocket | null = null
  private subscriptions: Map<string, TelemetrySubscription[]> = new Map()
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectInterval = 5000

  constructor(private wsUrl: string = 'ws://localhost:8084/ws/telemetry') {}

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(this.wsUrl)

        this.ws.onopen = () => {
          console.info('Telemetry service connected')
          this.reconnectAttempts = 0
          resolve()
        }

        this.ws.onmessage = (event) => {
          try {
            const data: TelemetryData = JSON.parse(event.data)
            this.handleTelemetryData(data)
          } catch (error) {
            console.error('Error parsing telemetry data:', error)
          }
        }

        this.ws.onclose = () => {
          console.warn('Telemetry service disconnected')
          this.attemptReconnect()
        }

        this.ws.onerror = (error) => {
          console.error('Telemetry service error:', error)
          reject(error)
        }

      } catch (error) {
        reject(error)
      }
    })
  }

  private handleTelemetryData(data: TelemetryData) {
    const vehicleSubscriptions = this.subscriptions.get(data.vehicleId) || []
    
    vehicleSubscriptions.forEach(subscription => {
      try {
        subscription.callback(data)
      } catch (error) {
        console.error('Error in telemetry callback:', error)
      }
    })
  }

  private attemptReconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++
      setTimeout(() => {
        console.info(`Attempting to reconnect telemetry service (${this.reconnectAttempts}/${this.maxReconnectAttempts})`)
        this.connect().catch(console.error)
      }, this.reconnectInterval)
    }
  }

  subscribe(subscription: TelemetrySubscription): () => void {
    const { vehicleId } = subscription
    
    if (!this.subscriptions.has(vehicleId)) {
      this.subscriptions.set(vehicleId, [])
    }
    
    this.subscriptions.get(vehicleId)!.push(subscription)

    // Send subscription message to server
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        type: 'subscribe',
        vehicleId,
        filters: subscription.filters
      }))
    }

    // Return unsubscribe function
    return () => {
      const subs = this.subscriptions.get(vehicleId) || []
      const index = subs.indexOf(subscription)
      if (index > -1) {
        subs.splice(index, 1)
        
        if (subs.length === 0) {
          this.subscriptions.delete(vehicleId)
          
          // Send unsubscribe message to server
          if (this.ws?.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({
              type: 'unsubscribe',
              vehicleId
            }))
          }
        }
      }
    }
  }

  // Abu Dhabi specific telemetry filtering
  subscribeToAbuDhabiVehicles(callback: (data: TelemetryData) => void): () => void {
    const abuDhabiBounds = {
      north: 24.6,
      south: 24.2,
      east: 55.0,
      west: 54.0
    }

    return this.subscribe({
      vehicleId: '*', // Subscribe to all vehicles
      callback: (data) => {
        const { lat, lng } = data.location
        if (lat >= abuDhabiBounds.south && lat <= abuDhabiBounds.north &&
            lng >= abuDhabiBounds.west && lng <= abuDhabiBounds.east) {
          callback(data)
        }
      }
    })
  }

  disconnect() {
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
    this.subscriptions.clear()
  }

  // Get historical telemetry data
  async getHistoricalData(vehicleId: string, startTime: number, endTime: number): Promise<TelemetryData[]> {
    try {
      const response = await fetch(`/api/telemetry/historical`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          vehicleId,
          startTime,
          endTime
        })
      })

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`)
      }

      return await response.json()
    } catch (error) {
      console.error('Error fetching historical telemetry data:', error)
      throw error
    }
  }
}

// Singleton instance
export const telemetryService = new TelemetryService()

// React hook for telemetry data
export const useTelemetry = (vehicleId: string) => {
  const [data, setData] = useState<TelemetryData | null>(null)
  const [connected, setConnected] = useState(false)

  useEffect(() => {
    let unsubscribe: (() => void) | null = null

    const initTelemetry = async () => {
      try {
        await telemetryService.connect()
        setConnected(true)

        unsubscribe = telemetryService.subscribe({
          vehicleId,
          callback: setData
        })
      } catch (error) {
        console.error('Failed to connect to telemetry service:', error)
        setConnected(false)
      }
    }

    initTelemetry()

    return () => {
      if (unsubscribe) {
        unsubscribe()
      }
    }
  }, [vehicleId])

  return { data, connected }
}

export default telemetryService

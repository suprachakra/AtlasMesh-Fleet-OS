import { useState, useEffect, useCallback, useRef } from 'react'

// Abu Dhabi coordinates
export const ABU_DHABI_CENTER = {
  lat: 24.4539,
  lng: 54.3773,
  zoom: 12
}

export type MapProvider = 'openstreetmap' | 'google' | 'satellite'

export interface MapProviderConfig {
  provider: MapProvider
  apiKey?: string
  style?: string
  center: { lat: number; lng: number }
  zoom: number
}

export interface MapInstance {
  provider: MapProvider
  map: any
  isLoaded: boolean
  error: string | null
  switchProvider: (provider: MapProvider) => void
  addMarker: (id: string, lat: number, lng: number, options?: any) => void
  removeMarker: (id: string) => void
  addRoute: (id: string, coordinates: Array<{lat: number, lng: number}>, options?: any) => void
  removeRoute: (id: string) => void
  addGeofence: (id: string, geometry: any, options?: any) => void
  removeGeofence: (id: string) => void
  fitBounds: (bounds: any) => void
  setCenter: (lat: number, lng: number, zoom?: number) => void
  getCenter: () => { lat: number; lng: number }
  getZoom: () => number
}

interface UseMapProviderOptions {
  container: HTMLElement | null
  initialProvider?: MapProvider
  center?: { lat: number; lng: number }
  zoom?: number
  onProviderChange?: (provider: MapProvider) => void
  onMapReady?: (map: any) => void
  onError?: (error: string) => void
}

export const useMapProvider = ({
  container,
  initialProvider = 'openstreetmap',
  center = ABU_DHABI_CENTER,
  zoom = ABU_DHABI_CENTER.zoom,
  onProviderChange,
  onMapReady,
  onError
}: UseMapProviderOptions): MapInstance => {
  const [currentProvider, setCurrentProvider] = useState<MapProvider>(initialProvider)
  const [isLoaded, setIsLoaded] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const mapRef = useRef<any>(null)
  const markersRef = useRef<Map<string, any>>(new Map())
  const routesRef = useRef<Map<string, any>>(new Map())
  const geofencesRef = useRef<Map<string, any>>(new Map())

  // Initialize map based on provider
  const initializeMap = useCallback(async (provider: MapProvider) => {
    if (!container) return

    try {
      setIsLoaded(false)
      setError(null)

      // Clean up existing map
      if (mapRef.current) {
        if (currentProvider === 'google' && mapRef.current.setMap) {
          mapRef.current.setMap(null)
        } else if (mapRef.current.remove) {
          mapRef.current.remove()
        }
        mapRef.current = null
      }

      // Clear existing overlays
      markersRef.current.clear()
      routesRef.current.clear()
      geofencesRef.current.clear()

      switch (provider) {
        case 'openstreetmap':
          await initializeOpenStreetMap()
          break
        case 'google':
          await initializeGoogleMaps()
          break
        case 'satellite':
          await initializeGoogleMapsSatellite()
          break
        default:
          throw new Error(`Unsupported map provider: ${provider}`)
      }

      setCurrentProvider(provider)
      setIsLoaded(true)
      onProviderChange?.(provider)
      onMapReady?.(mapRef.current)
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to initialize map'
      setError(errorMessage)
      onError?.(errorMessage)
    }
  }, [container, onProviderChange, onMapReady, onError])

  // Initialize OpenStreetMap using Leaflet
  const initializeOpenStreetMap = useCallback(async () => {
    // Dynamically import Leaflet to avoid SSR issues
    const L = await import('leaflet')
    
    // Fix for default markers in Leaflet
    delete (L.Icon.Default.prototype as any)._getIconUrl
    L.Icon.Default.mergeOptions({
      iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
      iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
      shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
    })

    const map = L.map(container).setView([center.lat, center.lng], zoom)

    // Add OpenStreetMap tiles
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors',
      maxZoom: 19,
    }).addTo(map)

    mapRef.current = map
  }, [container, center, zoom])

  // Initialize Google Maps
  const initializeGoogleMaps = useCallback(async () => {
    // Load Google Maps API if not already loaded
    if (!window.google) {
      await loadGoogleMapsAPI()
    }

    const map = new window.google.maps.Map(container, {
      center: { lat: center.lat, lng: center.lng },
      zoom: zoom,
      mapTypeId: 'roadmap',
      styles: [
        // Custom styling for better AV fleet visibility
        {
          featureType: 'road',
          elementType: 'geometry',
          stylers: [{ color: '#f5f5f5' }]
        },
        {
          featureType: 'road.highway',
          elementType: 'geometry',
          stylers: [{ color: '#dadada' }]
        }
      ]
    })

    mapRef.current = map
  }, [container, center, zoom])

  // Initialize Google Maps Satellite
  const initializeGoogleMapsSatellite = useCallback(async () => {
    if (!window.google) {
      await loadGoogleMapsAPI()
    }

    const map = new window.google.maps.Map(container, {
      center: { lat: center.lat, lng: center.lng },
      zoom: zoom,
      mapTypeId: 'hybrid', // Satellite with labels
    })

    mapRef.current = map
  }, [container, center, zoom])

  // Load Google Maps API dynamically
  const loadGoogleMapsAPI = useCallback((): Promise<void> => {
    return new Promise((resolve, reject) => {
      if (window.google) {
        resolve()
        return
      }

      const script = document.createElement('script')
      script.src = `https://maps.googleapis.com/maps/api/js?key=${process.env.REACT_APP_GOOGLE_MAPS_API_KEY || 'demo-key'}&libraries=geometry,places`
      script.async = true
      script.defer = true
      script.onload = () => resolve()
      script.onerror = () => reject(new Error('Failed to load Google Maps API'))
      document.head.appendChild(script)
    })
  }, [])

  // Switch between map providers
  const switchProvider = useCallback((provider: MapProvider) => {
    initializeMap(provider)
  }, [initializeMap])

  // Add marker to map
  const addMarker = useCallback((id: string, lat: number, lng: number, options: any = {}) => {
    if (!mapRef.current) return

    let marker: any

    switch (currentProvider) {
      case 'openstreetmap':
        // Leaflet marker
        const L = window.L
        marker = L.marker([lat, lng], {
          icon: options.icon || L.icon({
            iconUrl: options.iconUrl || 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
            iconSize: options.iconSize || [25, 41],
            iconAnchor: options.iconAnchor || [12, 41],
            popupAnchor: options.popupAnchor || [1, -34],
          })
        }).addTo(mapRef.current)

        if (options.popup) {
          marker.bindPopup(options.popup)
        }
        break

      case 'google':
      case 'satellite':
        // Google Maps marker
        marker = new window.google.maps.Marker({
          position: { lat, lng },
          map: mapRef.current,
          title: options.title || '',
          icon: options.icon || {
            url: options.iconUrl || '/icons/vehicle-marker.png',
            scaledSize: new window.google.maps.Size(32, 32),
            anchor: new window.google.maps.Point(16, 32)
          }
        })

        if (options.popup) {
          const infoWindow = new window.google.maps.InfoWindow({
            content: options.popup
          })
          marker.addListener('click', () => {
            infoWindow.open(mapRef.current, marker)
          })
        }
        break
    }

    if (marker) {
      markersRef.current.set(id, marker)
    }
  }, [currentProvider])

  // Remove marker from map
  const removeMarker = useCallback((id: string) => {
    const marker = markersRef.current.get(id)
    if (!marker) return

    switch (currentProvider) {
      case 'openstreetmap':
        marker.remove()
        break
      case 'google':
      case 'satellite':
        marker.setMap(null)
        break
    }

    markersRef.current.delete(id)
  }, [currentProvider])

  // Add route to map
  const addRoute = useCallback((id: string, coordinates: Array<{lat: number, lng: number}>, options: any = {}) => {
    if (!mapRef.current || coordinates.length < 2) return

    let route: any

    switch (currentProvider) {
      case 'openstreetmap':
        const L = window.L
        const latLngs = coordinates.map(coord => [coord.lat, coord.lng])
        route = L.polyline(latLngs, {
          color: options.color || '#3B82F6',
          weight: options.weight || 4,
          opacity: options.opacity || 0.8
        }).addTo(mapRef.current)
        break

      case 'google':
      case 'satellite':
        route = new window.google.maps.Polyline({
          path: coordinates,
          geodesic: true,
          strokeColor: options.color || '#3B82F6',
          strokeOpacity: options.opacity || 0.8,
          strokeWeight: options.weight || 4,
          map: mapRef.current
        })
        break
    }

    if (route) {
      routesRef.current.set(id, route)
    }
  }, [currentProvider])

  // Remove route from map
  const removeRoute = useCallback((id: string) => {
    const route = routesRef.current.get(id)
    if (!route) return

    switch (currentProvider) {
      case 'openstreetmap':
        route.remove()
        break
      case 'google':
      case 'satellite':
        route.setMap(null)
        break
    }

    routesRef.current.delete(id)
  }, [currentProvider])

  // Add geofence to map
  const addGeofence = useCallback((id: string, geometry: any, options: any = {}) => {
    if (!mapRef.current) return

    let geofence: any

    switch (currentProvider) {
      case 'openstreetmap':
        const L = window.L
        if (geometry.type === 'circle') {
          geofence = L.circle([geometry.center.lat, geometry.center.lng], {
            radius: geometry.radius,
            color: options.strokeColor || '#EF4444',
            fillColor: options.fillColor || '#EF4444',
            fillOpacity: options.fillOpacity || 0.2,
            weight: options.strokeWeight || 2
          }).addTo(mapRef.current)
        } else if (geometry.type === 'polygon') {
          const latLngs = geometry.coordinates.map((coord: any) => [coord.lat, coord.lng])
          geofence = L.polygon(latLngs, {
            color: options.strokeColor || '#EF4444',
            fillColor: options.fillColor || '#EF4444',
            fillOpacity: options.fillOpacity || 0.2,
            weight: options.strokeWeight || 2
          }).addTo(mapRef.current)
        }
        break

      case 'google':
      case 'satellite':
        if (geometry.type === 'circle') {
          geofence = new window.google.maps.Circle({
            center: geometry.center,
            radius: geometry.radius,
            strokeColor: options.strokeColor || '#EF4444',
            strokeOpacity: options.strokeOpacity || 0.8,
            strokeWeight: options.strokeWeight || 2,
            fillColor: options.fillColor || '#EF4444',
            fillOpacity: options.fillOpacity || 0.2,
            map: mapRef.current
          })
        } else if (geometry.type === 'polygon') {
          geofence = new window.google.maps.Polygon({
            paths: geometry.coordinates,
            strokeColor: options.strokeColor || '#EF4444',
            strokeOpacity: options.strokeOpacity || 0.8,
            strokeWeight: options.strokeWeight || 2,
            fillColor: options.fillColor || '#EF4444',
            fillOpacity: options.fillOpacity || 0.2,
            map: mapRef.current
          })
        }
        break
    }

    if (geofence) {
      geofencesRef.current.set(id, geofence)
    }
  }, [currentProvider])

  // Remove geofence from map
  const removeGeofence = useCallback((id: string) => {
    const geofence = geofencesRef.current.get(id)
    if (!geofence) return

    switch (currentProvider) {
      case 'openstreetmap':
        geofence.remove()
        break
      case 'google':
      case 'satellite':
        geofence.setMap(null)
        break
    }

    geofencesRef.current.delete(id)
  }, [currentProvider])

  // Fit map to bounds
  const fitBounds = useCallback((bounds: any) => {
    if (!mapRef.current) return

    switch (currentProvider) {
      case 'openstreetmap':
        mapRef.current.fitBounds(bounds)
        break
      case 'google':
      case 'satellite':
        mapRef.current.fitBounds(bounds)
        break
    }
  }, [currentProvider])

  // Set map center
  const setCenter = useCallback((lat: number, lng: number, newZoom?: number) => {
    if (!mapRef.current) return

    switch (currentProvider) {
      case 'openstreetmap':
        mapRef.current.setView([lat, lng], newZoom || mapRef.current.getZoom())
        break
      case 'google':
      case 'satellite':
        mapRef.current.setCenter({ lat, lng })
        if (newZoom !== undefined) {
          mapRef.current.setZoom(newZoom)
        }
        break
    }
  }, [currentProvider])

  // Get map center
  const getCenter = useCallback(() => {
    if (!mapRef.current) return center

    switch (currentProvider) {
      case 'openstreetmap':
        const leafletCenter = mapRef.current.getCenter()
        return { lat: leafletCenter.lat, lng: leafletCenter.lng }
      case 'google':
      case 'satellite':
        const googleCenter = mapRef.current.getCenter()
        return { lat: googleCenter.lat(), lng: googleCenter.lng() }
      default:
        return center
    }
  }, [currentProvider, center])

  // Get map zoom
  const getZoom = useCallback(() => {
    if (!mapRef.current) return zoom

    return mapRef.current.getZoom()
  }, [zoom])

  // Initialize map on mount
  useEffect(() => {
    if (container) {
      initializeMap(currentProvider)
    }

    // Cleanup on unmount
    return () => {
      if (mapRef.current) {
        if (currentProvider === 'google' && mapRef.current.setMap) {
          mapRef.current.setMap(null)
        } else if (mapRef.current.remove) {
          mapRef.current.remove()
        }
      }
    }
  }, [container, initializeMap, currentProvider])

  return {
    provider: currentProvider,
    map: mapRef.current,
    isLoaded,
    error,
    switchProvider,
    addMarker,
    removeMarker,
    addRoute,
    removeRoute,
    addGeofence,
    removeGeofence,
    fitBounds,
    setCenter,
    getCenter,
    getZoom
  }
}

// Type declarations for global objects
declare global {
  interface Window {
    google: any
    L: any
  }
}

import '@testing-library/jest-dom'
import { expect, afterEach, beforeAll, afterAll } from 'vitest'
import { cleanup } from '@testing-library/react'
import { server } from './mocks/server'

// Extend Vitest's expect with jest-dom matchers
expect.extend({})

// Mock IntersectionObserver
global.IntersectionObserver = class IntersectionObserver {
  constructor() {}
  disconnect() {}
  observe() {}
  unobserve() {}
}

// Mock ResizeObserver
global.ResizeObserver = class ResizeObserver {
  constructor() {}
  disconnect() {}
  observe() {}
  unobserve() {}
}

// Mock matchMedia
Object.defineProperty(window, 'matchMedia', {
  writable: true,
  value: (query: string) => ({
    matches: false,
    media: query,
    onchange: null,
    addListener: () => {},
    removeListener: () => {},
    addEventListener: () => {},
    removeEventListener: () => {},
    dispatchEvent: () => {},
  }),
})

// Mock scrollTo
Object.defineProperty(window, 'scrollTo', {
  writable: true,
  value: () => {},
})

// Mock HTMLCanvasElement.getContext for map components
HTMLCanvasElement.prototype.getContext = () => null

// Mock Mapbox GL
vi.mock('mapbox-gl', () => ({
  Map: vi.fn(() => ({
    on: vi.fn(),
    off: vi.fn(),
    remove: vi.fn(),
    addSource: vi.fn(),
    removeSource: vi.fn(),
    addLayer: vi.fn(),
    removeLayer: vi.fn(),
    getSource: vi.fn(),
    getLayer: vi.fn(),
    flyTo: vi.fn(),
    fitBounds: vi.fn(),
    setCenter: vi.fn(),
    setZoom: vi.fn(),
    getCenter: vi.fn(),
    getZoom: vi.fn(),
    getBounds: vi.fn(),
    project: vi.fn(),
    unproject: vi.fn(),
  })),
  Marker: vi.fn(() => ({
    setLngLat: vi.fn().mockReturnThis(),
    addTo: vi.fn().mockReturnThis(),
    remove: vi.fn(),
    setPopup: vi.fn().mockReturnThis(),
  })),
  Popup: vi.fn(() => ({
    setLngLat: vi.fn().mockReturnThis(),
    setHTML: vi.fn().mockReturnThis(),
    addTo: vi.fn().mockReturnThis(),
    remove: vi.fn(),
  })),
  NavigationControl: vi.fn(),
  ScaleControl: vi.fn(),
  AttributionControl: vi.fn(),
}))

// Mock WebRTC
global.RTCPeerConnection = vi.fn(() => ({
  createOffer: vi.fn(),
  createAnswer: vi.fn(),
  setLocalDescription: vi.fn(),
  setRemoteDescription: vi.fn(),
  addIceCandidate: vi.fn(),
  close: vi.fn(),
  addEventListener: vi.fn(),
  removeEventListener: vi.fn(),
}))

global.navigator.mediaDevices = {
  getUserMedia: vi.fn(() => Promise.resolve({
    getTracks: () => [],
    getVideoTracks: () => [],
    getAudioTracks: () => [],
  })),
  enumerateDevices: vi.fn(() => Promise.resolve([])),
  getSupportedConstraints: vi.fn(() => ({})),
} as any

// Mock WebSocket
global.WebSocket = vi.fn(() => ({
  send: vi.fn(),
  close: vi.fn(),
  addEventListener: vi.fn(),
  removeEventListener: vi.fn(),
  readyState: 1,
  CONNECTING: 0,
  OPEN: 1,
  CLOSING: 2,
  CLOSED: 3,
})) as any

// Mock geolocation
global.navigator.geolocation = {
  getCurrentPosition: vi.fn((success) => 
    success({
      coords: {
        latitude: 37.7749,
        longitude: -122.4194,
        accuracy: 10,
        altitude: null,
        altitudeAccuracy: null,
        heading: null,
        speed: null,
      },
      timestamp: Date.now(),
    })
  ),
  watchPosition: vi.fn(),
  clearWatch: vi.fn(),
}

// Mock localStorage
const localStorageMock = {
  getItem: vi.fn(),
  setItem: vi.fn(),
  removeItem: vi.fn(),
  clear: vi.fn(),
  length: 0,
  key: vi.fn(),
}
Object.defineProperty(window, 'localStorage', {
  value: localStorageMock,
})

// Mock sessionStorage
const sessionStorageMock = {
  getItem: vi.fn(),
  setItem: vi.fn(),
  removeItem: vi.fn(),
  clear: vi.fn(),
  length: 0,
  key: vi.fn(),
}
Object.defineProperty(window, 'sessionStorage', {
  value: sessionStorageMock,
})

// Setup MSW
beforeAll(() => server.listen({ onUnhandledRequest: 'error' }))
afterEach(() => {
  server.resetHandlers()
  cleanup()
  vi.clearAllMocks()
})
afterAll(() => server.close())

// Custom render function for testing
export * from '@testing-library/react'
export { default as userEvent } from '@testing-library/user-event'

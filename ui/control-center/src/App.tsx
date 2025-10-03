import React, { Suspense } from 'react'
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import { QueryClient, QueryClientProvider } from 'react-query'
import { ReactQueryDevtools } from 'react-query/devtools'
import { Provider } from 'react-redux'
import { PersistGate } from 'redux-persist/integration/react'
import { HelmetProvider } from 'react-helmet-async'
import { ErrorBoundary } from 'react-error-boundary'
import { Toaster } from 'react-hot-toast'

// Store
import { store, persistor } from '@store/index'

// Components
import { Layout } from '@components/Layout'
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { ErrorFallback } from '@components/ui/ErrorFallback'
import { AccessibilityProvider } from '@components/accessibility/AccessibilityProvider'

// Pages (lazy loaded for better performance) - Updated for 4-Module Architecture
const Dashboard = React.lazy(() => import('@pages/Dashboard'))
const OperationsCenter = React.lazy(() => import('@pages/OperationsCenter'))
const FleetScheduling = React.lazy(() => import('@pages/FleetScheduling'))
const VehicleManagement = React.lazy(() => import('@pages/VehicleManagement'))
const GarageManagement = React.lazy(() => import('@pages/GarageManagement'))
const Settings = React.lazy(() => import('@pages/Settings'))
const NotFound = React.lazy(() => import('@pages/NotFound'))

// Internationalization
import '@/i18n'

// Styles
import '@/styles/globals.css'

// React Query client configuration
const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      retry: 3,
      retryDelay: (attemptIndex) => Math.min(1000 * 2 ** attemptIndex, 30000),
      staleTime: 5 * 60 * 1000, // 5 minutes
      cacheTime: 10 * 60 * 1000, // 10 minutes
      refetchOnWindowFocus: false,
      refetchOnReconnect: 'always',
    },
    mutations: {
      retry: 1,
    },
  },
})

// Error boundary error handler
const handleError = (error: Error, errorInfo: { componentStack: string }) => {
  console.error('Application Error:', error, errorInfo)
  // In production, send to error reporting service
  if (import.meta.env.PROD) {
    // Example: Sentry.captureException(error, { contexts: { react: errorInfo } })
  }
}

function App() {
  return (
    <ErrorBoundary FallbackComponent={ErrorFallback} onError={handleError}>
      <HelmetProvider>
        <Provider store={store}>
          <PersistGate loading={<LoadingSpinner />} persistor={persistor}>
            <QueryClientProvider client={queryClient}>
              <AccessibilityProvider>
                <Router>
                  <div className="min-h-screen bg-gray-50 dark:bg-gray-900">
                    <Layout>
                      <Suspense fallback={<LoadingSpinner />}>
                        <Routes>
                          {/* Main Dashboard */}
                          <Route path="/" element={<Dashboard />} />
                          
                          {/* Module 1: Operations Command Center (Real-time Operations) */}
                          <Route path="/operations" element={<OperationsCenter />} />
                          <Route path="/operations/:section" element={<OperationsCenter />} />
                          
                          {/* Module 2: Planning & Scheduling (Resource Planning) */}
                          <Route path="/scheduling" element={<FleetScheduling />} />
                          <Route path="/scheduling/:view" element={<FleetScheduling />} />
                          
                          {/* Module 3: Fleet Management (Asset Lifecycle) */}
                          <Route path="/fleet" element={<VehicleManagement />} />
                          <Route path="/fleet/vehicle/:vehicleId" element={<VehicleManagement />} />
                          <Route path="/fleet/:view" element={<VehicleManagement />} />
                          
                          {/* Module 4: Garage PC Management (Infrastructure) */}
                          <Route path="/garage" element={<GarageManagement />} />
                          <Route path="/garage/node/:nodeId" element={<GarageManagement />} />
                          
                          {/* System Administration */}
                          <Route path="/admin" element={<Settings />} />
                          <Route path="/admin/:section" element={<Settings />} />
                          
                          {/* Legacy redirects for backward compatibility */}
                          <Route path="/live-ops" element={<OperationsCenter />} />
                          <Route path="/alerts" element={<OperationsCenter />} />
                          <Route path="/trips" element={<FleetScheduling />} />
                          <Route path="/vehicles" element={<VehicleManagement />} />
                          <Route path="/settings" element={<Settings />} />
                          
                          {/* 404 Not Found */}
                          <Route path="*" element={<NotFound />} />
                        </Routes>
                      </Suspense>
                    </Layout>
                    
                    {/* Global Toast Notifications */}
                    <Toaster
                      position="top-right"
                      toastOptions={{
                        duration: 5000,
                        className: 'focus-visible-ring',
                        style: {
                          background: 'var(--toast-bg)',
                          color: 'var(--toast-color)',
                          border: '1px solid var(--toast-border)',
                        },
                        success: {
                          iconTheme: {
                            primary: '#22c55e',
                            secondary: '#ffffff',
                          },
                        },
                        error: {
                          iconTheme: {
                            primary: '#ef4444',
                            secondary: '#ffffff',
                          },
                          duration: 8000, // Longer duration for errors
                        },
                      }}
                    />
                  </div>
                </Router>
              </AccessibilityProvider>
              
              {/* React Query DevTools (development only) */}
              {import.meta.env.DEV && <ReactQueryDevtools initialIsOpen={false} />}
            </QueryClientProvider>
          </PersistGate>
        </Provider>
      </HelmetProvider>
    </ErrorBoundary>
  )
}

export default App

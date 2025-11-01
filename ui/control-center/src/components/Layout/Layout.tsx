import React, { useState, useEffect } from 'react'
import { useLocation } from 'react-router-dom'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion, AnimatePresence } from 'framer-motion'

// Components
import { Sidebar } from './Sidebar'
import { Header } from './Header'
import { Breadcrumbs } from './Breadcrumbs'
import { NotificationCenter } from '@components/notifications/NotificationCenter'
import { CommandPalette } from '@components/ui/CommandPalette'
import { SkipLink } from '@components/accessibility/SkipLink'

// Hooks
import { useAuth } from '@hooks/useAuth'
import { useSector } from '@hooks/useSector'
import { useKeyboardShortcuts } from '@hooks/useKeyboardShortcuts'
import { useWebSocket } from '@hooks/useWebSocket'

// Types
import type { ReactNode } from 'react'

interface LayoutProps {
  children: ReactNode
}

export function Layout({ children }: LayoutProps) {
  const { t } = useTranslation()
  const location = useLocation()
  const { user, isAuthenticated } = useAuth()
  const { currentSector, sectorConfig } = useSector()
  
  // UI State
  const [sidebarOpen, setSidebarOpen] = useState(false)
  const [commandPaletteOpen, setCommandPaletteOpen] = useState(false)
  const [notificationCenterOpen, setNotificationCenterOpen] = useState(false)
  
  // WebSocket connection for real-time updates
  const { isConnected, connectionStatus } = useWebSocket()
  
  // Keyboard shortcuts
  useKeyboardShortcuts({
    'cmd+k': () => setCommandPaletteOpen(true),
    'cmd+/': () => setCommandPaletteOpen(true),
    'cmd+shift+n': () => setNotificationCenterOpen(true),
    'cmd+shift+s': () => setSidebarOpen(!sidebarOpen),
    'esc': () => {
      setCommandPaletteOpen(false)
      setNotificationCenterOpen(false)
    },
  })
  
  // Close sidebar on route change (mobile)
  useEffect(() => {
    setSidebarOpen(false)
  }, [location.pathname])
  
  // Generate page title based on route
  const getPageTitle = () => {
    const pathSegments = location.pathname.split('/').filter(Boolean)
    const baseTitle = 'AtlasMesh Control Center'
    
    if (pathSegments.length === 0) {
      return `${t('pages.dashboard')} - ${baseTitle}`
    }
    
    const pageKey = pathSegments[0]
    const pageTitle = t(`pages.${pageKey}`, { defaultValue: pageKey })
    return `${pageTitle} - ${baseTitle}`
  }
  
  // Generate page description
  const getPageDescription = () => {
    const pathSegments = location.pathname.split('/').filter(Boolean)
    if (pathSegments.length === 0) {
      return t('meta.dashboard.description')
    }
    
    const pageKey = pathSegments[0]
    return t(`meta.${pageKey}.description`, { 
      defaultValue: t('meta.default.description') 
    })
  }
  
  // Apply sector-specific theme
  useEffect(() => {
    if (sectorConfig?.theme) {
      document.documentElement.style.setProperty('--sector-primary', sectorConfig.theme.primary)
      document.documentElement.style.setProperty('--sector-secondary', sectorConfig.theme.secondary)
      document.documentElement.style.setProperty('--sector-accent', sectorConfig.theme.accent)
    }
  }, [sectorConfig])
  
  // If not authenticated, show minimal layout
  if (!isAuthenticated) {
    return (
      <>
        <Helmet>
          <title>{t('auth.signIn')} - AtlasMesh Control Center</title>
          <meta name="description" content={t('meta.auth.description')} />
        </Helmet>
        <div className="min-h-screen bg-gray-50 dark:bg-gray-900">
          {children}
        </div>
      </>
    )
  }
  
  return (
    <>
      <Helmet>
        <title>{getPageTitle()}</title>
        <meta name="description" content={getPageDescription()} />
        <meta name="viewport" content="width=device-width, initial-scale=1.0" />
        <meta name="theme-color" content={sectorConfig?.theme?.primary || '#1f2937'} />
        <link rel="canonical" href={`${window.location.origin}${location.pathname}`} />
      </Helmet>
      
      <div className={`min-h-screen bg-gray-50 dark:bg-gray-900 ${currentSector}`}>
        {/* Skip to main content link for screen readers */}
        <SkipLink href="#main-content" />
        
        {/* Sidebar */}
        <Sidebar 
          open={sidebarOpen} 
          onClose={() => setSidebarOpen(false)}
          currentSector={currentSector}
        />
        
        {/* Main content area */}
        <div className="lg:pl-72">
          {/* Header */}
          <Header
            onMenuClick={() => setSidebarOpen(true)}
            onNotificationClick={() => setNotificationCenterOpen(true)}
            onCommandPaletteClick={() => setCommandPaletteOpen(true)}
            connectionStatus={connectionStatus}
            user={user}
          />
          
          {/* Breadcrumbs */}
          <div className="px-4 sm:px-6 lg:px-8 py-4 border-b border-gray-200 dark:border-gray-700">
            <Breadcrumbs />
          </div>
          
          {/* Main content */}
          <main 
            id="main-content"
            className="px-4 sm:px-6 lg:px-8 py-6"
            role="main"
            aria-label={t('accessibility.mainContent')}
          >
            <AnimatePresence mode="wait">
              <motion.div
                key={location.pathname}
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: -20 }}
                transition={{ 
                  duration: 0.2,
                  ease: 'easeInOut'
                }}
                className="reduce-motion:transition-none"
              >
                {children}
              </motion.div>
            </AnimatePresence>
          </main>
        </div>
        
        {/* Command Palette */}
        <CommandPalette
          open={commandPaletteOpen}
          onClose={() => setCommandPaletteOpen(false)}
        />
        
        {/* Notification Center */}
        <NotificationCenter
          open={notificationCenterOpen}
          onClose={() => setNotificationCenterOpen(false)}
        />
        
        {/* Connection Status Indicator */}
        {!isConnected && (
          <div 
            className="fixed bottom-4 right-4 bg-warning-500 text-white px-4 py-2 rounded-lg shadow-lg z-50"
            role="alert"
            aria-live="polite"
          >
            <div className="flex items-center space-x-2">
              <div className="w-2 h-2 bg-white rounded-full animate-pulse" />
              <span className="text-sm font-medium">
                {t('connection.reconnecting')}
              </span>
            </div>
          </div>
        )}
        
        {/* Accessibility announcements */}
        <div
          id="accessibility-announcements"
          className="sr-only"
          aria-live="polite"
          aria-atomic="true"
        />
        
        {/* Loading overlay for page transitions */}
        <AnimatePresence>
          {/* This would be controlled by a global loading state */}
        </AnimatePresence>
      </div>
    </>
  )
}

// Performance optimization: memoize the layout to prevent unnecessary re-renders
export default React.memo(Layout)

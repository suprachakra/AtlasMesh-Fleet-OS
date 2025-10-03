import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react'
import { useTranslation } from 'react-i18next'

// Types
interface AccessibilitySettings {
  reducedMotion: boolean
  highContrast: boolean
  largeText: boolean
  screenReader: boolean
  keyboardNavigation: boolean
  announcements: boolean
}

interface AccessibilityContextType {
  settings: AccessibilitySettings
  updateSetting: (key: keyof AccessibilitySettings, value: boolean) => void
  announce: (message: string, priority?: 'polite' | 'assertive') => void
  focusElement: (selector: string) => void
  skipToContent: () => void
}

// Context
const AccessibilityContext = createContext<AccessibilityContextType | undefined>(undefined)

// Hook
export function useAccessibility() {
  const context = useContext(AccessibilityContext)
  if (!context) {
    throw new Error('useAccessibility must be used within an AccessibilityProvider')
  }
  return context
}

// Provider Props
interface AccessibilityProviderProps {
  children: ReactNode
}

export function AccessibilityProvider({ children }: AccessibilityProviderProps) {
  const { t } = useTranslation()
  
  // Initialize settings from system preferences and localStorage
  const [settings, setSettings] = useState<AccessibilitySettings>(() => {
    const saved = localStorage.getItem('accessibility-settings')
    const savedSettings = saved ? JSON.parse(saved) : {}
    
    return {
      reducedMotion: savedSettings.reducedMotion ?? window.matchMedia('(prefers-reduced-motion: reduce)').matches,
      highContrast: savedSettings.highContrast ?? window.matchMedia('(prefers-contrast: high)').matches,
      largeText: savedSettings.largeText ?? false,
      screenReader: savedSettings.screenReader ?? detectScreenReader(),
      keyboardNavigation: savedSettings.keyboardNavigation ?? true,
      announcements: savedSettings.announcements ?? true,
    }
  })
  
  // Update setting
  const updateSetting = (key: keyof AccessibilitySettings, value: boolean) => {
    setSettings(prev => {
      const newSettings = { ...prev, [key]: value }
      localStorage.setItem('accessibility-settings', JSON.stringify(newSettings))
      return newSettings
    })
  }
  
  // Announce message to screen readers
  const announce = (message: string, priority: 'polite' | 'assertive' = 'polite') => {
    if (!settings.announcements) return
    
    const announcer = document.getElementById('accessibility-announcements')
    if (announcer) {
      announcer.setAttribute('aria-live', priority)
      announcer.textContent = message
      
      // Clear after a delay to allow for re-announcements
      setTimeout(() => {
        announcer.textContent = ''
      }, 1000)
    }
  }
  
  // Focus element by selector
  const focusElement = (selector: string) => {
    const element = document.querySelector(selector) as HTMLElement
    if (element) {
      element.focus()
      // Announce focus change for screen readers
      if (settings.screenReader) {
        const label = element.getAttribute('aria-label') || 
                     element.getAttribute('title') || 
                     element.textContent?.trim() || 
                     t('accessibility.elementFocused')
        announce(t('accessibility.focusedOn', { element: label }))
      }
    }
  }
  
  // Skip to main content
  const skipToContent = () => {
    focusElement('#main-content')
  }
  
  // Apply accessibility settings to DOM
  useEffect(() => {
    const root = document.documentElement
    
    // Reduced motion
    if (settings.reducedMotion) {
      root.classList.add('reduce-motion')
    } else {
      root.classList.remove('reduce-motion')
    }
    
    // High contrast
    if (settings.highContrast) {
      root.classList.add('high-contrast')
    } else {
      root.classList.remove('high-contrast')
    }
    
    // Large text
    if (settings.largeText) {
      root.classList.add('large-text')
    } else {
      root.classList.remove('large-text')
    }
    
    // Keyboard navigation indicators
    if (settings.keyboardNavigation) {
      root.classList.add('keyboard-navigation')
    } else {
      root.classList.remove('keyboard-navigation')
    }
  }, [settings])
  
  // Listen for system preference changes
  useEffect(() => {
    const reducedMotionQuery = window.matchMedia('(prefers-reduced-motion: reduce)')
    const highContrastQuery = window.matchMedia('(prefers-contrast: high)')
    
    const handleReducedMotionChange = (e: MediaQueryListEvent) => {
      updateSetting('reducedMotion', e.matches)
    }
    
    const handleHighContrastChange = (e: MediaQueryListEvent) => {
      updateSetting('highContrast', e.matches)
    }
    
    reducedMotionQuery.addEventListener('change', handleReducedMotionChange)
    highContrastQuery.addEventListener('change', handleHighContrastChange)
    
    return () => {
      reducedMotionQuery.removeEventListener('change', handleReducedMotionChange)
      highContrastQuery.removeEventListener('change', handleHighContrastChange)
    }
  }, [])
  
  // Keyboard navigation detection
  useEffect(() => {
    let isUsingKeyboard = false
    
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Tab') {
        isUsingKeyboard = true
        document.body.classList.add('using-keyboard')
      }
    }
    
    const handleMouseDown = () => {
      isUsingKeyboard = false
      document.body.classList.remove('using-keyboard')
    }
    
    document.addEventListener('keydown', handleKeyDown)
    document.addEventListener('mousedown', handleMouseDown)
    
    return () => {
      document.removeEventListener('keydown', handleKeyDown)
      document.removeEventListener('mousedown', handleMouseDown)
    }
  }, [])
  
  // Focus management for route changes
  useEffect(() => {
    const handleRouteChange = () => {
      // Announce page change
      const pageTitle = document.title.split(' - ')[0]
      announce(t('accessibility.pageChanged', { page: pageTitle }), 'assertive')
      
      // Focus main content after route change
      setTimeout(() => {
        const mainContent = document.getElementById('main-content')
        if (mainContent) {
          mainContent.focus()
        }
      }, 100)
    }
    
    // Listen for route changes (this is a simplified approach)
    window.addEventListener('popstate', handleRouteChange)
    
    return () => {
      window.removeEventListener('popstate', handleRouteChange)
    }
  }, [announce, t])
  
  const value: AccessibilityContextType = {
    settings,
    updateSetting,
    announce,
    focusElement,
    skipToContent,
  }
  
  return (
    <AccessibilityContext.Provider value={value}>
      {children}
      
      {/* Global accessibility styles */}
      <style jsx global>{`
        /* Reduced motion styles */
        .reduce-motion *,
        .reduce-motion *::before,
        .reduce-motion *::after {
          animation-duration: 0.01ms !important;
          animation-iteration-count: 1 !important;
          transition-duration: 0.01ms !important;
          scroll-behavior: auto !important;
        }
        
        /* High contrast styles */
        .high-contrast {
          filter: contrast(150%);
        }
        
        .high-contrast button,
        .high-contrast input,
        .high-contrast select,
        .high-contrast textarea {
          border: 2px solid currentColor !important;
        }
        
        /* Large text styles */
        .large-text {
          font-size: 120% !important;
        }
        
        .large-text .text-xs { font-size: 0.9rem !important; }
        .large-text .text-sm { font-size: 1rem !important; }
        .large-text .text-base { font-size: 1.2rem !important; }
        .large-text .text-lg { font-size: 1.4rem !important; }
        .large-text .text-xl { font-size: 1.6rem !important; }
        
        /* Keyboard navigation styles */
        .using-keyboard *:focus {
          outline: 2px solid #3b82f6 !important;
          outline-offset: 2px !important;
        }
        
        /* Focus trap styles */
        .focus-trap {
          position: relative;
        }
        
        .focus-trap::before,
        .focus-trap::after {
          content: '';
          position: absolute;
          width: 1px;
          height: 1px;
          opacity: 0;
          pointer-events: none;
        }
        
        /* Skip link styles */
        .skip-link {
          position: absolute;
          top: -40px;
          left: 6px;
          background: #000;
          color: #fff;
          padding: 8px;
          text-decoration: none;
          border-radius: 4px;
          z-index: 1000;
          transition: top 0.3s;
        }
        
        .skip-link:focus {
          top: 6px;
        }
        
        /* Screen reader only content */
        .sr-only {
          position: absolute;
          width: 1px;
          height: 1px;
          padding: 0;
          margin: -1px;
          overflow: hidden;
          clip: rect(0, 0, 0, 0);
          white-space: nowrap;
          border: 0;
        }
        
        .sr-only-focusable:focus {
          position: static;
          width: auto;
          height: auto;
          padding: inherit;
          margin: inherit;
          overflow: visible;
          clip: auto;
          white-space: normal;
        }
        
        /* High contrast mode detection */
        @media (prefers-contrast: high) {
          .auto-high-contrast {
            filter: contrast(150%);
          }
        }
        
        /* Forced colors mode support */
        @media (forced-colors: active) {
          .forced-colors-adjust {
            forced-color-adjust: auto;
          }
        }
      `}</style>
    </AccessibilityContext.Provider>
  )
}

// Utility function to detect screen reader
function detectScreenReader(): boolean {
  // This is a simplified detection - in practice, you might use more sophisticated methods
  return (
    navigator.userAgent.includes('NVDA') ||
    navigator.userAgent.includes('JAWS') ||
    navigator.userAgent.includes('VoiceOver') ||
    window.speechSynthesis !== undefined
  )
}

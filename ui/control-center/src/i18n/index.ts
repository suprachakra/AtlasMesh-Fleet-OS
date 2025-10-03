import i18n from 'i18next'
import { initReactI18next } from 'react-i18next'
import LanguageDetector from 'i18next-browser-languagedetector'
import Backend from 'i18next-http-backend'

// Import translation files
import enTranslations from './locales/en.json'
import esTranslations from './locales/es.json'
import frTranslations from './locales/fr.json'
import arTranslations from './locales/ar.json'
import zhTranslations from './locales/zh.json'

// Supported languages
export const supportedLanguages = {
  en: 'English',
  es: 'Español',
  fr: 'Français',
  ar: 'العربية',
  zh: '中文',
} as const

export type SupportedLanguage = keyof typeof supportedLanguages

// RTL languages
export const rtlLanguages: SupportedLanguage[] = ['ar']

// Language resources
const resources = {
  en: { translation: enTranslations },
  es: { translation: esTranslations },
  fr: { translation: frTranslations },
  ar: { translation: arTranslations },
  zh: { translation: zhTranslations },
}

// Initialize i18next
i18n
  .use(Backend)
  .use(LanguageDetector)
  .use(initReactI18next)
  .init({
    // Resources
    resources,
    
    // Fallback language
    fallbackLng: 'en',
    
    // Debug mode (only in development)
    debug: import.meta.env.DEV,
    
    // Language detection options
    detection: {
      // Order of language detection methods
      order: [
        'localStorage',
        'sessionStorage',
        'navigator',
        'htmlTag',
        'path',
        'subdomain',
      ],
      
      // Cache user language
      caches: ['localStorage', 'sessionStorage'],
      
      // Exclude certain detection methods
      excludeCacheFor: ['cimode'],
      
      // Check for supported languages only
      checkWhitelist: true,
    },
    
    // Interpolation options
    interpolation: {
      escapeValue: false, // React already escapes values
      formatSeparator: ',',
      format: (value, format, lng) => {
        // Custom formatting functions
        if (format === 'uppercase') return value.toUpperCase()
        if (format === 'lowercase') return value.toLowerCase()
        if (format === 'capitalize') {
          return value.charAt(0).toUpperCase() + value.slice(1)
        }
        
        // Number formatting
        if (format === 'number') {
          return new Intl.NumberFormat(lng).format(value)
        }
        
        // Date formatting
        if (format === 'date') {
          return new Intl.DateTimeFormat(lng).format(new Date(value))
        }
        
        if (format === 'datetime') {
          return new Intl.DateTimeFormat(lng, {
            year: 'numeric',
            month: 'short',
            day: 'numeric',
            hour: '2-digit',
            minute: '2-digit',
          }).format(new Date(value))
        }
        
        // Currency formatting
        if (format?.startsWith('currency:')) {
          const currency = format.split(':')[1]
          return new Intl.NumberFormat(lng, {
            style: 'currency',
            currency: currency.toUpperCase(),
          }).format(value)
        }
        
        // Relative time formatting
        if (format === 'relative') {
          const rtf = new Intl.RelativeTimeFormat(lng, { numeric: 'auto' })
          const diff = (new Date(value).getTime() - Date.now()) / 1000
          
          if (Math.abs(diff) < 60) return rtf.format(Math.round(diff), 'second')
          if (Math.abs(diff) < 3600) return rtf.format(Math.round(diff / 60), 'minute')
          if (Math.abs(diff) < 86400) return rtf.format(Math.round(diff / 3600), 'hour')
          return rtf.format(Math.round(diff / 86400), 'day')
        }
        
        return value
      },
    },
    
    // React options
    react: {
      useSuspense: false, // We handle loading states manually
      bindI18n: 'languageChanged',
      bindI18nStore: '',
      transEmptyNodeValue: '',
      transSupportBasicHtmlNodes: true,
      transKeepBasicHtmlNodesFor: ['br', 'strong', 'i', 'em', 'span'],
    },
    
    // Backend options (for loading translations from server)
    backend: {
      loadPath: '/locales/{{lng}}.json',
      addPath: '/locales/add/{{lng}}',
      allowMultiLoading: false,
      crossDomain: false,
      withCredentials: false,
      requestOptions: {
        mode: 'cors',
        credentials: 'same-origin',
        cache: 'default',
      },
    },
    
    // Whitelist supported languages
    supportedLngs: Object.keys(supportedLanguages),
    
    // Load all namespaces
    ns: ['translation'],
    defaultNS: 'translation',
    
    // Key separator
    keySeparator: '.',
    nsSeparator: ':',
    
    // Pluralization
    pluralSeparator: '_',
    contextSeparator: '_',
    
    // Missing key handling
    saveMissing: import.meta.env.DEV,
    missingKeyHandler: (lng, ns, key, fallbackValue) => {
      if (import.meta.env.DEV) {
        console.warn(`Missing translation key: ${key} for language: ${lng}`)
      }
    },
    
    // Post-processing
    postProcess: ['interval', 'plural'],
  })

// Language change handler
i18n.on('languageChanged', (lng: string) => {
  // Update document language
  document.documentElement.lang = lng
  
  // Update document direction for RTL languages
  document.documentElement.dir = rtlLanguages.includes(lng as SupportedLanguage) ? 'rtl' : 'ltr'
  
  // Update CSS custom properties for language-specific styling
  document.documentElement.style.setProperty('--text-direction', 
    rtlLanguages.includes(lng as SupportedLanguage) ? 'rtl' : 'ltr'
  )
  
  // Announce language change to screen readers
  const announcer = document.getElementById('accessibility-announcements')
  if (announcer) {
    announcer.textContent = `Language changed to ${supportedLanguages[lng as SupportedLanguage] || lng}`
  }
})

// Custom hooks for i18n
export const useLanguage = () => {
  const currentLanguage = i18n.language as SupportedLanguage
  const isRTL = rtlLanguages.includes(currentLanguage)
  
  const changeLanguage = (lng: SupportedLanguage) => {
    i18n.changeLanguage(lng)
  }
  
  return {
    currentLanguage,
    isRTL,
    changeLanguage,
    supportedLanguages,
  }
}

// Utility functions
export const formatMessage = (key: string, values?: Record<string, any>) => {
  return i18n.t(key, values)
}

export const formatNumber = (value: number, lng?: string) => {
  return new Intl.NumberFormat(lng || i18n.language).format(value)
}

export const formatCurrency = (value: number, currency = 'USD', lng?: string) => {
  return new Intl.NumberFormat(lng || i18n.language, {
    style: 'currency',
    currency,
  }).format(value)
}

export const formatDate = (date: Date | string, options?: Intl.DateTimeFormatOptions, lng?: string) => {
  return new Intl.DateTimeFormat(lng || i18n.language, options).format(new Date(date))
}

export const formatRelativeTime = (date: Date | string, lng?: string) => {
  const rtf = new Intl.RelativeTimeFormat(lng || i18n.language, { numeric: 'auto' })
  const diff = (new Date(date).getTime() - Date.now()) / 1000
  
  if (Math.abs(diff) < 60) return rtf.format(Math.round(diff), 'second')
  if (Math.abs(diff) < 3600) return rtf.format(Math.round(diff / 60), 'minute')
  if (Math.abs(diff) < 86400) return rtf.format(Math.round(diff / 3600), 'hour')
  return rtf.format(Math.round(diff / 86400), 'day')
}

// Pluralization helper
export const pluralize = (count: number, key: string, values?: Record<string, any>) => {
  return i18n.t(key, { count, ...values })
}

export default i18n

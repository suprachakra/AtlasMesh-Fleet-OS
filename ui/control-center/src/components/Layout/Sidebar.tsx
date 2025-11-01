import React, { Fragment } from 'react'
import { NavLink, useLocation } from 'react-router-dom'
import { useTranslation } from 'react-i18next'
import { Dialog, Transition } from '@headlessui/react'
import { 
  XMarkIcon,
  HomeIcon,
  TruckIcon,
  MapIcon,
  BellIcon,
  CogIcon,
  ChartBarIcon,
  ExclamationTriangleIcon,
  ShieldCheckIcon,
  WrenchScrewdriverIcon,
  BuildingOfficeIcon,
  UserGroupIcon,
} from '@heroicons/react/24/outline'
import { clsx } from 'clsx'

// Components
import { SectorBadge } from '@components/ui/SectorBadge'
import { Logo } from '@components/ui/Logo'

// Hooks
import { useSector } from '@hooks/useSector'
import { useAuth } from '@hooks/useAuth'
import { useNotifications } from '@hooks/useNotifications'

// Types
import type { Sector } from '@types/sector'

interface SidebarProps {
  open: boolean
  onClose: () => void
  currentSector: Sector
}

interface NavigationItem {
  name: string
  href: string
  icon: React.ComponentType<{ className?: string }>
  current?: boolean
  badge?: number
  disabled?: boolean
  sectors?: Sector[]
}

export function Sidebar({ open, onClose, currentSector }: SidebarProps) {
  const { t } = useTranslation()
  const location = useLocation()
  const { user, permissions } = useAuth()
  const { sectorConfig } = useSector()
  const { unreadCount } = useNotifications()
  
  // Navigation items - Updated for 4-Module Architecture
  const navigation: NavigationItem[] = [
    {
      name: t('navigation.dashboard'),
      href: '/',
      icon: HomeIcon,
      current: location.pathname === '/',
    },
    {
      name: 'Operations Center',
      href: '/operations',
      icon: ChartBarIcon,
      current: location.pathname.startsWith('/operations') || location.pathname.startsWith('/live-ops') || location.pathname.startsWith('/alerts'),
      badge: unreadCount,
    },
    {
      name: 'Planning & Scheduling',
      href: '/scheduling',
      icon: MapIcon,
      current: location.pathname.startsWith('/scheduling') || location.pathname.startsWith('/trips'),
    },
    {
      name: 'Fleet Management',
      href: '/fleet',
      icon: TruckIcon,
      current: location.pathname.startsWith('/fleet') || location.pathname.startsWith('/vehicles'),
    },
    {
      name: 'Garage PC',
      href: '/garage',
      icon: BuildingOfficeIcon,
      current: location.pathname.startsWith('/garage'),
      disabled: !permissions.includes('infrastructure:read'),
    },
    // System Administration
    {
      name: 'System Admin',
      href: '/admin',
      icon: CogIcon,
      current: location.pathname.startsWith('/admin') || location.pathname.startsWith('/settings'),
      disabled: !permissions.includes('admin:read'),
    },
    // Sector-specific navigation items (kept for backward compatibility)
    {
      name: t('navigation.security'),
      href: '/security',
      icon: ShieldCheckIcon,
      current: location.pathname.startsWith('/security'),
      sectors: ['defense'],
      disabled: !permissions.includes('security:read'),
    },
    {
      name: t('navigation.maintenance'),
      href: '/maintenance',
      icon: WrenchScrewdriverIcon,
      current: location.pathname.startsWith('/maintenance'),
      sectors: ['mining', 'logistics'],
      disabled: !permissions.includes('maintenance:read'),
    },
    {
      name: t('navigation.dispatch'),
      href: '/dispatch',
      icon: BuildingOfficeIcon,
      current: location.pathname.startsWith('/dispatch'),
      sectors: ['logistics', 'ridehail'],
      disabled: !permissions.includes('dispatch:read'),
    },
    {
      name: t('navigation.passengers'),
      href: '/passengers',
      icon: UserGroupIcon,
      current: location.pathname.startsWith('/passengers'),
      sectors: ['ridehail'],
      disabled: !permissions.includes('passengers:read'),
    },
    {
      name: t('navigation.settings'),
      href: '/settings',
      icon: CogIcon,
      current: location.pathname.startsWith('/settings'),
    },
  ]
  
  // Filter navigation items based on current sector and permissions
  const filteredNavigation = navigation.filter(item => {
    // If item has sector restrictions, check if current sector is included
    if (item.sectors && !item.sectors.includes(currentSector)) {
      return false
    }
    
    // Don't show disabled items unless user has override permissions
    if (item.disabled && !permissions.includes('admin:override')) {
      return false
    }
    
    return true
  })
  
  const SidebarContent = () => (
    <div className="flex h-full flex-col">
      {/* Logo and sector badge */}
      <div className="flex h-16 shrink-0 items-center px-6 border-b border-gray-200 dark:border-gray-700">
        <Logo className="h-8 w-auto" />
        <div className="ml-auto">
          <SectorBadge sector={currentSector} size="sm" />
        </div>
      </div>
      
      {/* Navigation */}
      <nav className="flex flex-1 flex-col px-6 py-6" aria-label={t('accessibility.primaryNavigation')}>
        <ul role="list" className="flex flex-1 flex-col gap-y-7">
          <li>
            <ul role="list" className="-mx-2 space-y-1">
              {filteredNavigation.map((item) => (
                <li key={item.name}>
                  <NavLink
                    to={item.href}
                    className={({ isActive }) =>
                      clsx(
                        'group flex gap-x-3 rounded-md p-2 text-sm leading-6 font-semibold transition-colors duration-200',
                        'focus-visible-ring',
                        isActive || item.current
                          ? 'bg-primary-50 text-primary-700 dark:bg-primary-900/20 dark:text-primary-300'
                          : 'text-gray-700 hover:text-primary-700 hover:bg-gray-50 dark:text-gray-300 dark:hover:text-primary-300 dark:hover:bg-gray-800',
                        item.disabled && 'opacity-50 cursor-not-allowed'
                      )
                    }
                    onClick={item.disabled ? (e) => e.preventDefault() : undefined}
                    aria-current={item.current ? 'page' : undefined}
                    aria-disabled={item.disabled}
                  >
                    <item.icon
                      className={clsx(
                        'h-6 w-6 shrink-0',
                        item.current
                          ? 'text-primary-700 dark:text-primary-300'
                          : 'text-gray-400 group-hover:text-primary-700 dark:group-hover:text-primary-300'
                      )}
                      aria-hidden="true"
                    />
                    <span className="truncate">{item.name}</span>
                    {item.badge && item.badge > 0 && (
                      <span
                        className={clsx(
                          'ml-auto inline-flex items-center justify-center px-2 py-1 text-xs font-bold leading-none rounded-full',
                          item.current
                            ? 'bg-primary-100 text-primary-700 dark:bg-primary-800 dark:text-primary-200'
                            : 'bg-error-100 text-error-700 dark:bg-error-900/20 dark:text-error-400'
                        )}
                        aria-label={t('accessibility.unreadNotifications', { count: item.badge })}
                      >
                        {item.badge > 99 ? '99+' : item.badge}
                      </span>
                    )}
                  </NavLink>
                </li>
              ))}
            </ul>
          </li>
          
          {/* User info and status */}
          <li className="mt-auto">
            <div className="border-t border-gray-200 dark:border-gray-700 pt-6">
              <div className="flex items-center gap-x-4 px-2 py-3 text-sm font-semibold leading-6 text-gray-900 dark:text-gray-100">
                <div className="h-8 w-8 rounded-full bg-primary-500 flex items-center justify-center">
                  <span className="text-sm font-medium text-white">
                    {user?.name?.charAt(0)?.toUpperCase() || 'U'}
                  </span>
                </div>
                <div className="flex-1 min-w-0">
                  <p className="truncate">{user?.name || t('user.unknown')}</p>
                  <p className="text-xs text-gray-500 dark:text-gray-400 truncate">
                    {user?.role || t('user.operator')}
                  </p>
                </div>
              </div>
              
              {/* System status indicators */}
              <div className="px-2 py-2 space-y-2">
                <div className="flex items-center justify-between text-xs">
                  <span className="text-gray-500 dark:text-gray-400">
                    {t('status.system')}
                  </span>
                  <div className="flex items-center space-x-1">
                    <div className="w-2 h-2 bg-success-500 rounded-full" />
                    <span className="text-success-700 dark:text-success-400">
                      {t('status.operational')}
                    </span>
                  </div>
                </div>
                
                <div className="flex items-center justify-between text-xs">
                  <span className="text-gray-500 dark:text-gray-400">
                    {t('status.sector')}
                  </span>
                  <span className="text-gray-700 dark:text-gray-300 capitalize">
                    {sectorConfig?.displayName || currentSector}
                  </span>
                </div>
              </div>
            </div>
          </li>
        </ul>
      </nav>
    </div>
  )
  
  return (
    <>
      {/* Mobile sidebar */}
      <Transition.Root show={open} as={Fragment}>
        <Dialog as="div" className="relative z-50 lg:hidden" onClose={onClose}>
          <Transition.Child
            as={Fragment}
            enter="transition-opacity ease-linear duration-300"
            enterFrom="opacity-0"
            enterTo="opacity-100"
            leave="transition-opacity ease-linear duration-300"
            leaveFrom="opacity-100"
            leaveTo="opacity-0"
          >
            <div className="fixed inset-0 bg-gray-900/80 backdrop-blur-sm" />
          </Transition.Child>
          
          <div className="fixed inset-0 flex">
            <Transition.Child
              as={Fragment}
              enter="transition ease-in-out duration-300 transform"
              enterFrom="-translate-x-full"
              enterTo="translate-x-0"
              leave="transition ease-in-out duration-300 transform"
              leaveFrom="translate-x-0"
              leaveTo="-translate-x-full"
            >
              <Dialog.Panel className="relative mr-16 flex w-full max-w-xs flex-1">
                <Transition.Child
                  as={Fragment}
                  enter="ease-in-out duration-300"
                  enterFrom="opacity-0"
                  enterTo="opacity-100"
                  leave="ease-in-out duration-300"
                  leaveFrom="opacity-100"
                  leaveTo="opacity-0"
                >
                  <div className="absolute left-full top-0 flex w-16 justify-center pt-5">
                    <button
                      type="button"
                      className="-m-2.5 p-2.5 text-white hover:text-gray-300 focus-visible-ring"
                      onClick={onClose}
                      aria-label={t('accessibility.closeSidebar')}
                    >
                      <XMarkIcon className="h-6 w-6" aria-hidden="true" />
                    </button>
                  </div>
                </Transition.Child>
                
                <div className="flex grow flex-col gap-y-5 overflow-y-auto bg-white dark:bg-gray-900 shadow-xl">
                  <SidebarContent />
                </div>
              </Dialog.Panel>
            </Transition.Child>
          </div>
        </Dialog>
      </Transition.Root>
      
      {/* Desktop sidebar */}
      <div className="hidden lg:fixed lg:inset-y-0 lg:z-50 lg:flex lg:w-72 lg:flex-col">
        <div className="flex grow flex-col gap-y-5 overflow-y-auto border-r border-gray-200 dark:border-gray-700 bg-white dark:bg-gray-900 shadow-sm">
          <SidebarContent />
        </div>
      </div>
    </>
  )
}

export default React.memo(Sidebar)


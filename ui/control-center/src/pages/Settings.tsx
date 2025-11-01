import React, { useState, useEffect } from 'react'
import { useParams } from 'react-router-dom'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  CogIcon,
  UserIcon,
  ShieldCheckIcon,
  BellIcon,
  GlobeAltIcon,
  PaintBrushIcon,
  ServerIcon,
  KeyIcon,
  DocumentTextIcon,
  ChartBarIcon,
} from '@heroicons/react/24/outline'

// Components
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { ErrorMessage } from '@components/ui/ErrorMessage'
import { Button } from '@components/ui/Button'
import { Input } from '@components/ui/Input'
import { Select } from '@components/ui/Select'
import { Switch } from '@components/ui/Switch'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@components/ui/Tabs'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Badge } from '@components/ui/Badge'

// Hooks
import { useSettings } from '@hooks/useSettings'
import { useAuth } from '@hooks/useAuth'
import { useSector } from '@hooks/useSector'

// Types
import type { UserSettings, SystemSettings, NotificationSettings } from '@types/settings'

const settingsSections = [
  { id: 'profile', label: 'Profile', icon: UserIcon },
  { id: 'notifications', label: 'Notifications', icon: BellIcon },
  { id: 'appearance', label: 'Appearance', icon: PaintBrushIcon },
  { id: 'security', label: 'Security', icon: ShieldCheckIcon },
  { id: 'system', label: 'System', icon: ServerIcon },
  { id: 'api', label: 'API Keys', icon: KeyIcon },
  { id: 'reports', label: 'Reports', icon: ChartBarIcon },
  { id: 'compliance', label: 'Compliance', icon: DocumentTextIcon },
]

export default function Settings() {
  const { section } = useParams<{ section?: string }>()
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { currentSector, sectorConfig } = useSector()

  // State
  const [activeSection, setActiveSection] = useState(section || 'profile')
  const [saving, setSaving] = useState(false)
  const [unsavedChanges, setUnsavedChanges] = useState(false)

  // Data fetching
  const {
    data: settings,
    isLoading,
    error,
    refetch,
    updateSettings,
  } = useSettings()

  // Local state for form data
  const [formData, setFormData] = useState<any>({})

  useEffect(() => {
    if (settings) {
      setFormData(settings)
    }
  }, [settings])

  useEffect(() => {
    if (section) {
      setActiveSection(section)
    }
  }, [section])

  const handleInputChange = (field: string, value: any) => {
    setFormData((prev: any) => ({
      ...prev,
      [field]: value,
    }))
    setUnsavedChanges(true)
  }

  const handleSave = async () => {
    setSaving(true)
    try {
      await updateSettings(formData)
      setUnsavedChanges(false)
    } catch (error) {
      console.error('Failed to save settings:', error)
    } finally {
      setSaving(false)
    }
  }

  const handleReset = () => {
    setFormData(settings)
    setUnsavedChanges(false)
  }

  if (isLoading) {
    return <LoadingSpinner />
  }

  if (error) {
    return <ErrorMessage error={error} onRetry={refetch} />
  }

  return (
    <>
      <Helmet>
        <title>{t('pages.settings')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="space-y-6">
        {/* Header */}
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
          <div>
            <h1 className="text-2xl font-bold text-gray-900 dark:text-white">
              {t('pages.settings')}
            </h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('settings.description')}
            </p>
          </div>
          
          {unsavedChanges && (
            <div className="mt-4 sm:mt-0 flex items-center space-x-3">
              <Button
                variant="secondary"
                size="sm"
                onClick={handleReset}
                disabled={saving}
              >
                {t('actions.reset')}
              </Button>
              <Button
                variant="primary"
                size="sm"
                onClick={handleSave}
                loading={saving}
              >
                {t('actions.save')}
              </Button>
            </div>
          )}
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
          {/* Sidebar Navigation */}
          <div className="lg:col-span-1">
            <Card>
              <CardContent className="p-0">
                <nav className="space-y-1">
                  {settingsSections.map((item) => {
                    const Icon = item.icon
                    const isActive = activeSection === item.id
                    const hasPermission = permissions.includes(`settings:${item.id}`) || item.id === 'profile'
                    
                    if (!hasPermission) return null
                    
                    return (
                      <button
                        key={item.id}
                        onClick={() => setActiveSection(item.id)}
                        className={`w-full flex items-center px-3 py-2 text-sm font-medium rounded-md transition-colors ${
                          isActive
                            ? 'bg-blue-100 text-blue-700 dark:bg-blue-900 dark:text-blue-300'
                            : 'text-gray-600 hover:text-gray-900 hover:bg-gray-50 dark:text-gray-400 dark:hover:text-gray-200 dark:hover:bg-gray-800'
                        }`}
                      >
                        <Icon className="mr-3 h-5 w-5" />
                        {t(`settings.sections.${item.label.toLowerCase()}`)}
                      </button>
                    )
                  })}
                </nav>
              </CardContent>
            </Card>
          </div>

          {/* Settings Content */}
          <div className="lg:col-span-3">
            <Card>
              <CardContent className="p-6">
                {activeSection === 'profile' && (
                  <div className="space-y-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 dark:text-white">
                        {t('settings.profile.title')}
                      </h3>
                      <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                        {t('settings.profile.description')}
                      </p>
                    </div>

                    <div className="grid grid-cols-1 gap-6 sm:grid-cols-2">
                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.profile.firstName')}
                        </label>
                        <Input
                          type="text"
                          value={formData.firstName || ''}
                          onChange={(e) => handleInputChange('firstName', e.target.value)}
                          className="mt-1"
                        />
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.profile.lastName')}
                        </label>
                        <Input
                          type="text"
                          value={formData.lastName || ''}
                          onChange={(e) => handleInputChange('lastName', e.target.value)}
                          className="mt-1"
                        />
                      </div>

                      <div className="sm:col-span-2">
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.profile.email')}
                        </label>
                        <Input
                          type="email"
                          value={formData.email || ''}
                          onChange={(e) => handleInputChange('email', e.target.value)}
                          className="mt-1"
                        />
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.profile.role')}
                        </label>
                        <Input
                          type="text"
                          value={user?.role || ''}
                          disabled
                          className="mt-1 bg-gray-50 dark:bg-gray-800"
                        />
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.profile.sector')}
                        </label>
                        <Input
                          type="text"
                          value={currentSector}
                          disabled
                          className="mt-1 bg-gray-50 dark:bg-gray-800"
                        />
                      </div>
                    </div>
                  </div>
                )}

                {activeSection === 'notifications' && (
                  <div className="space-y-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 dark:text-white">
                        {t('settings.notifications.title')}
                      </h3>
                      <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                        {t('settings.notifications.description')}
                      </p>
                    </div>

                    <div className="space-y-4">
                      <div className="flex items-center justify-between">
                        <div>
                          <h4 className="text-sm font-medium text-gray-900 dark:text-white">
                            {t('settings.notifications.criticalAlerts')}
                          </h4>
                          <p className="text-sm text-gray-500 dark:text-gray-400">
                            {t('settings.notifications.criticalAlertsDesc')}
                          </p>
                        </div>
                        <Switch
                          checked={formData.notifications?.criticalAlerts ?? true}
                          onChange={(checked) => handleInputChange('notifications.criticalAlerts', checked)}
                        />
                      </div>

                      <div className="flex items-center justify-between">
                        <div>
                          <h4 className="text-sm font-medium text-gray-900 dark:text-white">
                            {t('settings.notifications.tripUpdates')}
                          </h4>
                          <p className="text-sm text-gray-500 dark:text-gray-400">
                            {t('settings.notifications.tripUpdatesDesc')}
                          </p>
                        </div>
                        <Switch
                          checked={formData.notifications?.tripUpdates ?? true}
                          onChange={(checked) => handleInputChange('notifications.tripUpdates', checked)}
                        />
                      </div>

                      <div className="flex items-center justify-between">
                        <div>
                          <h4 className="text-sm font-medium text-gray-900 dark:text-white">
                            {t('settings.notifications.maintenanceReminders')}
                          </h4>
                          <p className="text-sm text-gray-500 dark:text-gray-400">
                            {t('settings.notifications.maintenanceRemindersDesc')}
                          </p>
                        </div>
                        <Switch
                          checked={formData.notifications?.maintenanceReminders ?? true}
                          onChange={(checked) => handleInputChange('notifications.maintenanceReminders', checked)}
                        />
                      </div>

                      <div className="flex items-center justify-between">
                        <div>
                          <h4 className="text-sm font-medium text-gray-900 dark:text-white">
                            {t('settings.notifications.systemUpdates')}
                          </h4>
                          <p className="text-sm text-gray-500 dark:text-gray-400">
                            {t('settings.notifications.systemUpdatesDesc')}
                          </p>
                        </div>
                        <Switch
                          checked={formData.notifications?.systemUpdates ?? false}
                          onChange={(checked) => handleInputChange('notifications.systemUpdates', checked)}
                        />
                      </div>
                    </div>
                  </div>
                )}

                {activeSection === 'appearance' && (
                  <div className="space-y-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 dark:text-white">
                        {t('settings.appearance.title')}
                      </h3>
                      <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                        {t('settings.appearance.description')}
                      </p>
                    </div>

                    <div className="space-y-4">
                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.appearance.theme')}
                        </label>
                        <Select
                          value={formData.appearance?.theme || 'system'}
                          onChange={(value) => handleInputChange('appearance.theme', value)}
                          options={[
                            { value: 'light', label: t('settings.appearance.themes.light') },
                            { value: 'dark', label: t('settings.appearance.themes.dark') },
                            { value: 'system', label: t('settings.appearance.themes.system') },
                          ]}
                          className="mt-1"
                        />
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.appearance.language')}
                        </label>
                        <Select
                          value={formData.appearance?.language || 'en'}
                          onChange={(value) => handleInputChange('appearance.language', value)}
                          options={[
                            { value: 'en', label: 'English' },
                            { value: 'es', label: 'Español' },
                            { value: 'fr', label: 'Français' },
                            { value: 'de', label: 'Deutsch' },
                            { value: 'zh', label: '中文' },
                          ]}
                          className="mt-1"
                        />
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          {t('settings.appearance.timezone')}
                        </label>
                        <Select
                          value={formData.appearance?.timezone || 'UTC'}
                          onChange={(value) => handleInputChange('appearance.timezone', value)}
                          options={[
                            { value: 'UTC', label: 'UTC' },
                            { value: 'America/New_York', label: 'Eastern Time' },
                            { value: 'America/Chicago', label: 'Central Time' },
                            { value: 'America/Denver', label: 'Mountain Time' },
                            { value: 'America/Los_Angeles', label: 'Pacific Time' },
                            { value: 'Europe/London', label: 'London' },
                            { value: 'Europe/Berlin', label: 'Berlin' },
                            { value: 'Asia/Tokyo', label: 'Tokyo' },
                          ]}
                          className="mt-1"
                        />
                      </div>
                    </div>
                  </div>
                )}

                {activeSection === 'security' && (
                  <div className="space-y-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 dark:text-white">
                        {t('settings.security.title')}
                      </h3>
                      <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                        {t('settings.security.description')}
                      </p>
                    </div>

                    <div className="space-y-4">
                      <div className="flex items-center justify-between">
                        <div>
                          <h4 className="text-sm font-medium text-gray-900 dark:text-white">
                            {t('settings.security.twoFactor')}
                          </h4>
                          <p className="text-sm text-gray-500 dark:text-gray-400">
                            {t('settings.security.twoFactorDesc')}
                          </p>
                        </div>
                        <Badge variant={formData.security?.twoFactorEnabled ? 'success' : 'secondary'}>
                          {formData.security?.twoFactorEnabled ? t('common.enabled') : t('common.disabled')}
                        </Badge>
                      </div>

                      <div>
                        <Button
                          variant="secondary"
                          size="sm"
                          onClick={() => {/* Implement 2FA setup */}}
                        >
                          {formData.security?.twoFactorEnabled 
                            ? t('settings.security.disable2FA') 
                            : t('settings.security.enable2FA')
                          }
                        </Button>
                      </div>

                      <div className="border-t pt-4">
                        <h4 className="text-sm font-medium text-gray-900 dark:text-white mb-2">
                          {t('settings.security.changePassword')}
                        </h4>
                        <div className="space-y-3">
                          <Input
                            type="password"
                            placeholder={t('settings.security.currentPassword')}
                          />
                          <Input
                            type="password"
                            placeholder={t('settings.security.newPassword')}
                          />
                          <Input
                            type="password"
                            placeholder={t('settings.security.confirmPassword')}
                          />
                          <Button variant="secondary" size="sm">
                            {t('settings.security.updatePassword')}
                          </Button>
                        </div>
                      </div>
                    </div>
                  </div>
                )}

                {activeSection === 'system' && permissions.includes('settings:system') && (
                  <div className="space-y-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 dark:text-white">
                        {t('settings.system.title')}
                      </h3>
                      <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                        {t('settings.system.description')}
                      </p>
                    </div>

                    <div className="text-center py-8">
                      <ServerIcon className="mx-auto h-12 w-12 text-gray-400" />
                      <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
                        {t('settings.system.comingSoon')}
                      </h3>
                      <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                        {t('settings.system.comingSoonDesc')}
                      </p>
                    </div>
                  </div>
                )}

                {/* Add other sections as needed */}
                {!['profile', 'notifications', 'appearance', 'security', 'system'].includes(activeSection) && (
                  <div className="text-center py-8">
                    <CogIcon className="mx-auto h-12 w-12 text-gray-400" />
                    <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
                      {t('settings.comingSoon')}
                    </h3>
                    <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                      {t('settings.comingSoonDesc', { section: activeSection })}
                    </p>
                  </div>
                )}
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </>
  )
}

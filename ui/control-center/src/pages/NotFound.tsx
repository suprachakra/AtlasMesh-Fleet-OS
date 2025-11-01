import React from 'react'
import { Link } from 'react-router-dom'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  ExclamationTriangleIcon,
  HomeIcon,
  ArrowLeftIcon,
} from '@heroicons/react/24/outline'

// Components
import { Button } from '@components/ui/Button'

export default function NotFound() {
  const { t } = useTranslation()

  return (
    <>
      <Helmet>
        <title>{t('pages.notFound')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="min-h-screen flex items-center justify-center bg-gray-50 dark:bg-gray-900 py-12 px-4 sm:px-6 lg:px-8">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          className="max-w-md w-full space-y-8 text-center"
        >
          <div>
            <motion.div
              initial={{ scale: 0 }}
              animate={{ scale: 1 }}
              transition={{ delay: 0.2, type: "spring", stiffness: 200 }}
              className="mx-auto h-24 w-24 text-gray-400"
            >
              <ExclamationTriangleIcon className="h-full w-full" />
            </motion.div>
            
            <motion.h1
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.3 }}
              className="mt-6 text-6xl font-bold text-gray-900 dark:text-white"
            >
              404
            </motion.h1>
            
            <motion.h2
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.4 }}
              className="mt-2 text-3xl font-bold text-gray-900 dark:text-white"
            >
              {t('notFound.title')}
            </motion.h2>
            
            <motion.p
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.5 }}
              className="mt-4 text-lg text-gray-600 dark:text-gray-400"
            >
              {t('notFound.description')}
            </motion.p>
          </div>

          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.6 }}
            className="space-y-4"
          >
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Button
                as={Link}
                to="/"
                variant="primary"
                size="lg"
                icon={HomeIcon}
                className="w-full sm:w-auto"
              >
                {t('notFound.goHome')}
              </Button>
              
              <Button
                variant="secondary"
                size="lg"
                icon={ArrowLeftIcon}
                onClick={() => window.history.back()}
                className="w-full sm:w-auto"
              >
                {t('notFound.goBack')}
              </Button>
            </div>

            <div className="mt-8">
              <p className="text-sm text-gray-500 dark:text-gray-400">
                {t('notFound.helpText')}
              </p>
              <div className="mt-2 flex justify-center space-x-6 text-sm">
                <Link
                  to="/fleet"
                  className="text-blue-600 hover:text-blue-500 dark:text-blue-400 dark:hover:text-blue-300"
                >
                  {t('pages.fleet')}
                </Link>
                <Link
                  to="/trips"
                  className="text-blue-600 hover:text-blue-500 dark:text-blue-400 dark:hover:text-blue-300"
                >
                  {t('pages.trips')}
                </Link>
                <Link
                  to="/alerts"
                  className="text-blue-600 hover:text-blue-500 dark:text-blue-400 dark:hover:text-blue-300"
                >
                  {t('pages.alerts')}
                </Link>
                <Link
                  to="/settings"
                  className="text-blue-600 hover:text-blue-500 dark:text-blue-400 dark:hover:text-blue-300"
                >
                  {t('pages.settings')}
                </Link>
              </div>
            </div>
          </motion.div>

          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.8 }}
            className="mt-8 text-xs text-gray-400 dark:text-gray-500"
          >
            <p>AtlasMesh Fleet OS v2.0</p>
            <p className="mt-1">
              {t('notFound.errorCode')}: 404_PAGE_NOT_FOUND
            </p>
          </motion.div>
        </motion.div>
      </div>
    </>
  )
}

import React from 'react'
import { useTranslation } from 'react-i18next'

interface SkipLinkProps {
  href: string
  children?: React.ReactNode
}

export function SkipLink({ href, children }: SkipLinkProps) {
  const { t } = useTranslation()
  
  return (
    <a
      href={href}
      className="skip-link sr-only-focusable absolute top-0 left-0 z-50 bg-primary-600 text-white px-4 py-2 rounded-br-md focus:not-sr-only focus:top-0"
      onClick={(e) => {
        e.preventDefault()
        const target = document.querySelector(href)
        if (target) {
          target.focus()
          target.scrollIntoView({ behavior: 'smooth', block: 'start' })
        }
      }}
    >
      {children || t('accessibility.skipToContent')}
    </a>
  )
}

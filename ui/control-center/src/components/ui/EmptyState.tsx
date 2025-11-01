import React from 'react'

interface EmptyStateProps {
  title?: string
  description?: string
  icon?: 'no-data' | 'no-results' | 'no-events' | 'error'
}

export const EmptyState: React.FC<EmptyStateProps> = ({ 
  title = "Oops - no related data",
  description = "No data available at this time",
  icon = 'no-data'
}) => {
  return (
    <div className="flex flex-col items-center justify-center py-12">
      {/* Illustration */}
      <svg 
        className="w-32 h-32 mb-4" 
        viewBox="0 0 200 200" 
        fill="none" 
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Person with confused expression */}
        <circle cx="100" cy="80" r="30" fill="#E5E7EB" />
        <ellipse cx="90" cy="75" rx="4" ry="6" fill="#374151" />
        <ellipse cx="110" cy="75" rx="4" ry="6" fill="#374151" />
        <path d="M85 90 Q100 85 115 90" stroke="#374151" strokeWidth="2" fill="none" />
        
        {/* Body */}
        <rect x="75" y="110" width="50" height="60" rx="25" fill="#6366F1" />
        
        {/* Arms */}
        <path d="M75 120 L50 140" stroke="#6366F1" strokeWidth="12" strokeLinecap="round" />
        <path d="M125 120 L150 140" stroke="#6366F1" strokeWidth="12" strokeLinecap="round" />
        
        {/* Document icon */}
        <rect x="140" y="60" width="30" height="40" rx="2" fill="#FCD34D" />
        <line x1="145" y1="70" x2="165" y2="70" stroke="#92400E" strokeWidth="2" />
        <line x1="145" y1="80" x2="165" y2="80" stroke="#92400E" strokeWidth="2" />
        <line x1="145" y1="90" x2="160" y2="90" stroke="#92400E" strokeWidth="2" />
        
        {/* Sparkles/question marks */}
        <text x="60" y="50" fontSize="20" fill="#9CA3AF">?</text>
        <text x="135" y="45" fontSize="16" fill="#9CA3AF">?</text>
      </svg>

      {/* Text */}
      <h3 className="text-lg font-medium text-gray-900 mb-2">{title}</h3>
      <p className="text-gray-600 text-center max-w-sm">{description}</p>
    </div>
  )
}

export default EmptyState


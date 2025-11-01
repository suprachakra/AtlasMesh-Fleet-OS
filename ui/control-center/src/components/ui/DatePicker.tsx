import React, { useState } from 'react'
import { CalendarIcon } from '@heroicons/react/24/outline'
import { Input } from './Input'
import { Button } from './Button'
import { Modal } from './Modal'

interface DatePickerProps {
  value?: { start: string; end: string } | string
  onChange: (value: { start: string; end: string } | string) => void
  placeholder?: string
  range?: boolean
  disabled?: boolean
  className?: string
}

export function DatePicker({
  value,
  onChange,
  placeholder = 'Select date',
  range = false,
  disabled = false,
  className = '',
}: DatePickerProps) {
  const [isOpen, setIsOpen] = useState(false)
  const [tempValue, setTempValue] = useState(value)

  const formatDate = (date: string) => {
    if (!date) return ''
    return new Date(date).toLocaleDateString()
  }

  const getDisplayValue = () => {
    if (!value) return ''
    
    if (range && typeof value === 'object') {
      if (value.start && value.end) {
        return `${formatDate(value.start)} - ${formatDate(value.end)}`
      }
      if (value.start) {
        return `${formatDate(value.start)} - ...`
      }
      return ''
    }
    
    if (typeof value === 'string') {
      return formatDate(value)
    }
    
    return ''
  }

  const handleApply = () => {
    if (tempValue) {
      onChange(tempValue)
    }
    setIsOpen(false)
  }

  const handleClear = () => {
    const clearedValue = range ? { start: '', end: '' } : ''
    setTempValue(clearedValue)
    onChange(clearedValue)
    setIsOpen(false)
  }

  return (
    <>
      <div className={`relative ${className}`}>
        <Input
          value={getDisplayValue()}
          placeholder={placeholder}
          readOnly
          disabled={disabled}
          onClick={() => !disabled && setIsOpen(true)}
          className="cursor-pointer"
          icon={CalendarIcon}
        />
      </div>

      <Modal
        open={isOpen}
        onClose={() => setIsOpen(false)}
        title={range ? 'Select Date Range' : 'Select Date'}
      >
        <div className="space-y-4">
          {range ? (
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                  Start Date
                </label>
                <Input
                  type="date"
                  value={typeof tempValue === 'object' ? tempValue?.start || '' : ''}
                  onChange={(e) => setTempValue(prev => ({
                    start: e.target.value,
                    end: typeof prev === 'object' ? prev?.end || '' : ''
                  }))}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                  End Date
                </label>
                <Input
                  type="date"
                  value={typeof tempValue === 'object' ? tempValue?.end || '' : ''}
                  onChange={(e) => setTempValue(prev => ({
                    start: typeof prev === 'object' ? prev?.start || '' : '',
                    end: e.target.value
                  }))}
                />
              </div>
            </div>
          ) : (
            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                Date
              </label>
              <Input
                type="date"
                value={typeof tempValue === 'string' ? tempValue : ''}
                onChange={(e) => setTempValue(e.target.value)}
              />
            </div>
          )}

          <div className="flex justify-end space-x-3 pt-4 border-t">
            <Button variant="secondary" onClick={handleClear}>
              Clear
            </Button>
            <Button variant="secondary" onClick={() => setIsOpen(false)}>
              Cancel
            </Button>
            <Button variant="primary" onClick={handleApply}>
              Apply
            </Button>
          </div>
        </div>
      </Modal>
    </>
  )
}

import React, { useState, useMemo } from 'react'
import { cn } from '../../lib/utils'

// Generic data table component for AtlasMesh Fleet OS
// Reduces redundancy across vehicle lists, trip logs, incident reports, etc.

export interface Column<T> {
  key: keyof T | string
  header: string
  accessor?: (item: T) => React.ReactNode
  sortable?: boolean
  filterable?: boolean
  width?: string
  align?: 'left' | 'center' | 'right'
  className?: string
}

export interface DataTableProps<T> {
  data: T[]
  columns: Column<T>[]
  loading?: boolean
  error?: string
  emptyMessage?: string
  pageSize?: number
  showPagination?: boolean
  showSearch?: boolean
  showFilters?: boolean
  selectable?: boolean
  selectedItems?: T[]
  onSelectionChange?: (items: T[]) => void
  onRowClick?: (item: T) => void
  className?: string
  rowClassName?: (item: T) => string
  getRowId?: (item: T) => string | number
}

export type SortDirection = 'asc' | 'desc' | null

interface SortState {
  column: string | null
  direction: SortDirection
}

export function DataTable<T>({
  data,
  columns,
  loading = false,
  error,
  emptyMessage = 'No data available',
  pageSize = 10,
  showPagination = true,
  showSearch = true,
  showFilters = false,
  selectable = false,
  selectedItems = [],
  onSelectionChange,
  onRowClick,
  className,
  rowClassName,
  getRowId
}: DataTableProps<T>) {
  const [currentPage, setCurrentPage] = useState(1)
  const [searchTerm, setSearchTerm] = useState('')
  const [sortState, setSortState] = useState<SortState>({ column: null, direction: null })
  const [filters, setFilters] = useState<Record<string, string>>({})

  // Filter and search data
  const filteredData = useMemo(() => {
    let result = [...data]

    // Apply search
    if (searchTerm) {
      result = result.filter(item =>
        columns.some(column => {
          const value = column.accessor 
            ? column.accessor(item)
            : (item as any)[column.key]
          return String(value).toLowerCase().includes(searchTerm.toLowerCase())
        })
      )
    }

    // Apply column filters
    Object.entries(filters).forEach(([columnKey, filterValue]) => {
      if (filterValue) {
        result = result.filter(item => {
          const column = columns.find(col => col.key === columnKey)
          const value = column?.accessor 
            ? column.accessor(item)
            : (item as any)[columnKey]
          return String(value).toLowerCase().includes(filterValue.toLowerCase())
        })
      }
    })

    return result
  }, [data, searchTerm, filters, columns])

  // Sort data
  const sortedData = useMemo(() => {
    if (!sortState.column || !sortState.direction) {
      return filteredData
    }

    return [...filteredData].sort((a, b) => {
      const column = columns.find(col => col.key === sortState.column)
      const aValue = column?.accessor ? column.accessor(a) : (a as any)[sortState.column!]
      const bValue = column?.accessor ? column.accessor(b) : (b as any)[sortState.column!]

      let comparison = 0
      if (aValue < bValue) comparison = -1
      if (aValue > bValue) comparison = 1

      return sortState.direction === 'desc' ? -comparison : comparison
    })
  }, [filteredData, sortState, columns])

  // Paginate data
  const paginatedData = useMemo(() => {
    if (!showPagination) return sortedData
    
    const startIndex = (currentPage - 1) * pageSize
    return sortedData.slice(startIndex, startIndex + pageSize)
  }, [sortedData, currentPage, pageSize, showPagination])

  const totalPages = Math.ceil(sortedData.length / pageSize)

  // Handle sorting
  const handleSort = (columnKey: string) => {
    const column = columns.find(col => col.key === columnKey)
    if (!column?.sortable) return

    setSortState(prev => {
      if (prev.column === columnKey) {
        // Cycle through: asc -> desc -> null
        const direction = prev.direction === 'asc' ? 'desc' : prev.direction === 'desc' ? null : 'asc'
        return { column: direction ? columnKey : null, direction }
      }
      return { column: columnKey, direction: 'asc' }
    })
  }

  // Handle selection
  const handleSelectAll = (checked: boolean) => {
    if (!onSelectionChange) return
    onSelectionChange(checked ? paginatedData : [])
  }

  const handleSelectItem = (item: T, checked: boolean) => {
    if (!onSelectionChange) return
    
    const itemId = getRowId ? getRowId(item) : item
    const currentIds = selectedItems.map(selected => getRowId ? getRowId(selected) : selected)
    
    if (checked) {
      onSelectionChange([...selectedItems, item])
    } else {
      onSelectionChange(selectedItems.filter(selected => 
        (getRowId ? getRowId(selected) : selected) !== itemId
      ))
    }
  }

  const isItemSelected = (item: T) => {
    const itemId = getRowId ? getRowId(item) : item
    const selectedIds = selectedItems.map(selected => getRowId ? getRowId(selected) : selected)
    return selectedIds.includes(itemId)
  }

  const allSelected = paginatedData.length > 0 && paginatedData.every(item => isItemSelected(item))
  const someSelected = paginatedData.some(item => isItemSelected(item))

  if (error) {
    return (
      <div className="rounded-lg border border-red-200 bg-red-50 p-4">
        <div className="flex items-center gap-2 text-red-700">
          <span className="text-lg">⚠️</span>
          <span className="font-medium">Error loading data</span>
        </div>
        <p className="mt-1 text-sm text-red-600">{error}</p>
      </div>
    )
  }

  return (
    <div className={cn('space-y-4', className)}>
      {/* Search and Filters */}
      {(showSearch || showFilters) && (
        <div className="flex flex-col gap-4 sm:flex-row sm:items-center sm:justify-between">
          {showSearch && (
            <div className="relative">
              <input
                type="text"
                placeholder="Search..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="w-full rounded-md border border-gray-300 px-3 py-2 pl-10 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
              />
              <div className="absolute inset-y-0 left-0 flex items-center pl-3">
                <span className="text-gray-400">🔍</span>
              </div>
            </div>
          )}
          
          {showFilters && (
            <div className="flex gap-2">
              {columns.filter(col => col.filterable).map(column => (
                <input
                  key={String(column.key)}
                  type="text"
                  placeholder={`Filter ${column.header}...`}
                  value={filters[String(column.key)] || ''}
                  onChange={(e) => setFilters(prev => ({
                    ...prev,
                    [String(column.key)]: e.target.value
                  }))}
                  className="rounded-md border border-gray-300 px-3 py-2 text-sm focus:border-blue-500 focus:outline-none focus:ring-1 focus:ring-blue-500"
                />
              ))}
            </div>
          )}
        </div>
      )}

      {/* Table */}
      <div className="overflow-hidden rounded-lg border border-gray-200">
        <div className="overflow-x-auto">
          <table className="min-w-full divide-y divide-gray-200">
            <thead className="bg-gray-50">
              <tr>
                {selectable && (
                  <th className="w-12 px-6 py-3">
                    <input
                      type="checkbox"
                      checked={allSelected}
                      ref={(input) => {
                        if (input) input.indeterminate = someSelected && !allSelected
                      }}
                      onChange={(e) => handleSelectAll(e.target.checked)}
                      className="rounded border-gray-300 text-blue-600 focus:ring-blue-500"
                    />
                  </th>
                )}
                {columns.map((column) => (
                  <th
                    key={String(column.key)}
                    className={cn(
                      'px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500',
                      column.sortable && 'cursor-pointer hover:bg-gray-100',
                      column.align === 'center' && 'text-center',
                      column.align === 'right' && 'text-right',
                      column.className
                    )}
                    style={{ width: column.width }}
                    onClick={() => column.sortable && handleSort(String(column.key))}
                  >
                    <div className="flex items-center gap-1">
                      {column.header}
                      {column.sortable && (
                        <span className="text-gray-400">
                          {sortState.column === column.key
                            ? sortState.direction === 'asc' ? '↑' : '↓'
                            : '↕'
                          }
                        </span>
                      )}
                    </div>
                  </th>
                ))}
              </tr>
            </thead>
            <tbody className="divide-y divide-gray-200 bg-white">
              {loading ? (
                <tr>
                  <td colSpan={columns.length + (selectable ? 1 : 0)} className="px-6 py-12 text-center">
                    <div className="flex items-center justify-center gap-2 text-gray-500">
                      <div className="h-4 w-4 animate-spin rounded-full border-2 border-gray-300 border-t-blue-600"></div>
                      Loading...
                    </div>
                  </td>
                </tr>
              ) : paginatedData.length === 0 ? (
                <tr>
                  <td colSpan={columns.length + (selectable ? 1 : 0)} className="px-6 py-12 text-center text-gray-500">
                    {emptyMessage}
                  </td>
                </tr>
              ) : (
                paginatedData.map((item, index) => (
                  <tr
                    key={getRowId ? getRowId(item) : index}
                    className={cn(
                      'hover:bg-gray-50',
                      onRowClick && 'cursor-pointer',
                      rowClassName?.(item)
                    )}
                    onClick={() => onRowClick?.(item)}
                  >
                    {selectable && (
                      <td className="px-6 py-4">
                        <input
                          type="checkbox"
                          checked={isItemSelected(item)}
                          onChange={(e) => handleSelectItem(item, e.target.checked)}
                          onClick={(e) => e.stopPropagation()}
                          className="rounded border-gray-300 text-blue-600 focus:ring-blue-500"
                        />
                      </td>
                    )}
                    {columns.map((column) => (
                      <td
                        key={String(column.key)}
                        className={cn(
                          'px-6 py-4 text-sm text-gray-900',
                          column.align === 'center' && 'text-center',
                          column.align === 'right' && 'text-right',
                          column.className
                        )}
                      >
                        {column.accessor 
                          ? column.accessor(item)
                          : String((item as any)[column.key] || '')
                        }
                      </td>
                    ))}
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      </div>

      {/* Pagination */}
      {showPagination && totalPages > 1 && (
        <div className="flex items-center justify-between">
          <div className="text-sm text-gray-700">
            Showing {((currentPage - 1) * pageSize) + 1} to {Math.min(currentPage * pageSize, sortedData.length)} of {sortedData.length} results
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={() => setCurrentPage(prev => Math.max(1, prev - 1))}
              disabled={currentPage === 1}
              className="rounded-md border border-gray-300 px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50 disabled:cursor-not-allowed disabled:opacity-50"
            >
              Previous
            </button>
            
            <div className="flex items-center gap-1">
              {Array.from({ length: Math.min(5, totalPages) }, (_, i) => {
                const pageNum = i + 1
                return (
                  <button
                    key={pageNum}
                    onClick={() => setCurrentPage(pageNum)}
                    className={cn(
                      'rounded-md px-3 py-2 text-sm font-medium',
                      currentPage === pageNum
                        ? 'bg-blue-600 text-white'
                        : 'text-gray-700 hover:bg-gray-50'
                    )}
                  >
                    {pageNum}
                  </button>
                )
              })}
            </div>
            
            <button
              onClick={() => setCurrentPage(prev => Math.min(totalPages, prev + 1))}
              disabled={currentPage === totalPages}
              className="rounded-md border border-gray-300 px-3 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50 disabled:cursor-not-allowed disabled:opacity-50"
            >
              Next
            </button>
          </div>
        </div>
      )}
    </div>
  )
}

export default DataTable

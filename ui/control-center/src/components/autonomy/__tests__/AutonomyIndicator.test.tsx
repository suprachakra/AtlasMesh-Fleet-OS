import { describe, it, expect } from 'vitest'
import { render, screen } from '@test/utils/render'
import { AutonomyIndicator } from '../AutonomyIndicator'

describe('AutonomyIndicator', () => {
  it('renders L4 autonomy level correctly', () => {
    render(
      <AutonomyIndicator
        level="L4"
        status="active"
        confidence={0.95}
        oddCompliance={true}
      />
    )

    expect(screen.getByText('L4')).toBeInTheDocument()
    expect(screen.getByText('95%')).toBeInTheDocument()
  })

  it('shows intervention required badge when needed', () => {
    render(
      <AutonomyIndicator
        level="L3"
        status="degraded"
        confidence={0.65}
        interventionRequired={true}
      />
    )

    expect(screen.getByText(/intervention/i)).toBeInTheDocument()
  })

  it('displays ODD violation warning', () => {
    render(
      <AutonomyIndicator
        level="L4"
        status="active"
        oddCompliance={false}
      />
    )

    expect(screen.getByText(/odd/i)).toBeInTheDocument()
  })

  it('shows detailed information when showDetails is true', () => {
    render(
      <AutonomyIndicator
        level="L5"
        status="active"
        confidence={0.98}
        showDetails={true}
      />
    )

    expect(screen.getByText(/full automation/i)).toBeInTheDocument()
    expect(screen.getByText(/98.0%/)).toBeInTheDocument()
  })

  it('applies correct confidence color coding', () => {
    const { rerender } = render(
      <AutonomyIndicator
        level="L4"
        status="active"
        confidence={0.95}
        showDetails={true}
      />
    )

    // High confidence should be green
    expect(screen.getByText('95.0%')).toHaveClass('text-green-600')

    rerender(
      <AutonomyIndicator
        level="L4"
        status="active"
        confidence={0.75}
        showDetails={true}
      />
    )

    // Medium confidence should be yellow
    expect(screen.getByText('75.0%')).toHaveClass('text-yellow-600')

    rerender(
      <AutonomyIndicator
        level="L4"
        status="active"
        confidence={0.45}
        showDetails={true}
      />
    )

    // Low confidence should be red
    expect(screen.getByText('45.0%')).toHaveClass('text-red-600')
  })

  it('handles different autonomy levels', () => {
    const levels = ['L0', 'L1', 'L2', 'L3', 'L4', 'L5'] as const

    levels.forEach(level => {
      const { unmount } = render(
        <AutonomyIndicator
          level={level}
          status="active"
        />
      )

      expect(screen.getByText(level)).toBeInTheDocument()
      unmount()
    })
  })

  it('handles different status types', () => {
    const statuses = ['active', 'degraded', 'fallback', 'manual', 'offline'] as const

    statuses.forEach(status => {
      const { unmount } = render(
        <AutonomyIndicator
          level="L4"
          status={status}
        />
      )

      // Status should be displayed (exact text depends on i18n)
      expect(screen.getByRole('generic')).toBeInTheDocument()
      unmount()
    })
  })

  it('renders with different sizes', () => {
    const { rerender } = render(
      <AutonomyIndicator
        level="L4"
        status="active"
        size="sm"
      />
    )

    expect(screen.getByText('L4').closest('div')).toHaveClass('text-xs')

    rerender(
      <AutonomyIndicator
        level="L4"
        status="active"
        size="lg"
      />
    )

    expect(screen.getByText('L4').closest('div')).toHaveClass('text-base')
  })
})

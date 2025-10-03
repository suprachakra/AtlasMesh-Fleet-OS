import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent, waitFor } from '@test/utils/render'
import userEvent from '@testing-library/user-event'
import { EmergencyControls } from '../EmergencyControls'

// Mock hooks
vi.mock('@hooks/useAuth', () => ({
  useAuth: () => ({
    user: { id: 'user_123', name: 'Test User' },
    permissions: ['emergency:execute', 'incidents:create'],
  }),
}))

vi.mock('@hooks/useEmergencyProtocols', () => ({
  useEmergencyProtocols: () => ({
    executeEmergencyAction: vi.fn().mockResolvedValue({ success: true }),
    isExecuting: false,
  }),
}))

describe('EmergencyControls', () => {
  it('renders all emergency action buttons', () => {
    render(<EmergencyControls vehicleId="veh_001" />)

    expect(screen.getByText(/emergency stop/i)).toBeInTheDocument()
    expect(screen.getByText(/safe stop/i)).toBeInTheDocument()
    expect(screen.getByText(/request.*assistance/i)).toBeInTheDocument()
    expect(screen.getByText(/maintenance mode/i)).toBeInTheDocument()
    expect(screen.getByText(/create incident/i)).toBeInTheDocument()
  })

  it('opens confirmation modal for emergency stop', async () => {
    const user = userEvent.setup()
    render(<EmergencyControls vehicleId="veh_001" />)

    await user.click(screen.getByText(/emergency stop/i))

    expect(screen.getByText(/confirm/i)).toBeInTheDocument()
    expect(screen.getByText(/consequences/i)).toBeInTheDocument()
  })

  it('requires reason for emergency actions', async () => {
    const user = userEvent.setup()
    render(<EmergencyControls vehicleId="veh_001" />)

    await user.click(screen.getByText(/safe stop/i))

    // Should show reason selection
    expect(screen.getByText(/reason/i)).toBeInTheDocument()
    
    // Confirm button should be disabled without reason
    const confirmButton = screen.getByRole('button', { name: /confirm/i })
    expect(confirmButton).toBeDisabled()
  })

  it('enables confirm button after providing required information', async () => {
    const user = userEvent.setup()
    render(<EmergencyControls vehicleId="veh_001" />)

    await user.click(screen.getByText(/safe stop/i))

    // Fill in reason category
    const categorySelect = screen.getByRole('combobox')
    await user.click(categorySelect)
    await user.click(screen.getByText(/safety concern/i))

    // Fill in reason details
    const reasonInput = screen.getByPlaceholderText(/reason/i)
    await user.type(reasonInput, 'Test safety concern')

    // Confirm button should now be enabled
    const confirmButton = screen.getByRole('button', { name: /confirm/i })
    expect(confirmButton).not.toBeDisabled()
  })

  it('executes emergency action with correct parameters', async () => {
    const mockExecute = vi.fn().mockResolvedValue({ success: true })
    vi.mocked(require('@hooks/useEmergencyProtocols').useEmergencyProtocols).mockReturnValue({
      executeEmergencyAction: mockExecute,
      isExecuting: false,
    })

    const user = userEvent.setup()
    const onEmergencyAction = vi.fn()
    
    render(
      <EmergencyControls 
        vehicleId="veh_001" 
        tripId="trip_001"
        onEmergencyAction={onEmergencyAction}
      />
    )

    await user.click(screen.getByText(/safe stop/i))

    // Fill required fields
    const categorySelect = screen.getByRole('combobox')
    await user.click(categorySelect)
    await user.click(screen.getByText(/safety concern/i))

    const reasonInput = screen.getByPlaceholderText(/reason/i)
    await user.type(reasonInput, 'Test emergency')

    // Execute action
    await user.click(screen.getByRole('button', { name: /confirm/i }))

    await waitFor(() => {
      expect(mockExecute).toHaveBeenCalledWith(
        expect.objectContaining({
          actionId: 'safe_stop',
          vehicleId: 'veh_001',
          tripId: 'trip_001',
          reason: 'Test emergency',
          reasonCategory: 'safety_concern',
        })
      )
    })

    expect(onEmergencyAction).toHaveBeenCalledWith('safe_stop', expect.any(Object))
  })

  it('shows dual auth requirement for critical actions', async () => {
    const user = userEvent.setup()
    render(<EmergencyControls vehicleId="veh_001" />)

    await user.click(screen.getByText(/emergency stop/i))

    expect(screen.getByText(/dual.*auth/i)).toBeInTheDocument()
    expect(screen.getByPlaceholderText(/auth.*code/i)).toBeInTheDocument()
  })

  it('renders in different layouts', () => {
    const { rerender } = render(
      <EmergencyControls vehicleId="veh_001" layout="horizontal" />
    )

    expect(screen.getByText(/emergency stop/i).closest('div')).toHaveClass('flex')

    rerender(<EmergencyControls vehicleId="veh_001" layout="vertical" />)
    expect(screen.getByText(/emergency stop/i).closest('div')).toHaveClass('flex-col')

    rerender(<EmergencyControls vehicleId="veh_001" layout="grid" />)
    expect(screen.getByText(/emergency stop/i).closest('div')).toHaveClass('grid')
  })

  it('handles loading state during execution', async () => {
    vi.mocked(require('@hooks/useEmergencyProtocols').useEmergencyProtocols).mockReturnValue({
      executeEmergencyAction: vi.fn(),
      isExecuting: true,
    })

    render(<EmergencyControls vehicleId="veh_001" />)

    // All buttons should be disabled during execution
    const buttons = screen.getAllByRole('button')
    buttons.forEach(button => {
      expect(button).toBeDisabled()
    })
  })

  it('does not render actions without proper permissions', () => {
    vi.mocked(require('@hooks/useAuth').useAuth).mockReturnValue({
      user: { id: 'user_123', name: 'Test User' },
      permissions: [], // No permissions
    })

    render(<EmergencyControls vehicleId="veh_001" />)

    // Should not render any emergency action buttons
    expect(screen.queryByText(/emergency stop/i)).not.toBeInTheDocument()
    expect(screen.queryByText(/safe stop/i)).not.toBeInTheDocument()
  })

  it('allows incident creation with minimal permissions', () => {
    vi.mocked(require('@hooks/useAuth').useAuth).mockReturnValue({
      user: { id: 'user_123', name: 'Test User' },
      permissions: ['incidents:create'], // Only incident creation permission
    })

    render(<EmergencyControls vehicleId="veh_001" />)

    // Should render incident creation button
    expect(screen.getByText(/create incident/i)).toBeInTheDocument()
    
    // Should not render other emergency actions
    expect(screen.queryByText(/emergency stop/i)).not.toBeInTheDocument()
  })

  it('handles action execution errors gracefully', async () => {
    const mockExecute = vi.fn().mockRejectedValue(new Error('Network error'))
    vi.mocked(require('@hooks/useEmergencyProtocols').useEmergencyProtocols).mockReturnValue({
      executeEmergencyAction: mockExecute,
      isExecuting: false,
    })

    const user = userEvent.setup()
    render(<EmergencyControls vehicleId="veh_001" />)

    await user.click(screen.getByText(/request.*assistance/i))

    // Fill required reason
    const reasonInput = screen.getByPlaceholderText(/reason/i)
    await user.type(reasonInput, 'Test request')

    // Execute action
    await user.click(screen.getByRole('button', { name: /confirm/i }))

    // Should handle error gracefully (modal should remain open)
    await waitFor(() => {
      expect(mockExecute).toHaveBeenCalled()
    })

    // Modal should still be visible (error handling)
    expect(screen.getByText(/request.*assistance/i)).toBeInTheDocument()
  })
})
